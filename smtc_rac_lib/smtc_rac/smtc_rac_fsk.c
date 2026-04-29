/**
 * @file      smtc_rac_fsk.c
 *
 * @brief     smtc_rac_fsk api implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>   // memset

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include "smtc_rac_lbt.h"
#include "smtc_rac_local_func.h"

// Conditional logging for FSK module
#if RAC_FSK_LOG_ENABLE
#define RAC_LOG_APP_PREFIX "RAC-FSK"
#include "smtc_rac_log.h"
#else
// Disabled logging - all macros become no-ops
#define RAC_LOG_ERROR( ... ) \
    do                       \
    {                        \
    } while( 0 )
#define RAC_LOG_WARN( ... ) \
    do                      \
    {                       \
    } while( 0 )
#define RAC_LOG_INFO( ... ) \
    do                      \
    {                       \
    } while( 0 )
#define RAC_LOG_DEBUG( ... ) \
    do                       \
    {                        \
    } while( 0 )
#define RAC_LOG_CONFIG( ... ) \
    do                        \
    {                         \
    } while( 0 )
#define RAC_LOG_TX( ... ) \
    do                    \
    {                     \
    } while( 0 )
#define RAC_LOG_RX( ... ) \
    do                    \
    {                     \
    } while( 0 )
#define RAC_LOG_STATS( ... ) \
    do                       \
    {                        \
    } while( 0 )
#endif
#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#elif defined( LR11XX )
#include "ralf_lr11xx.h"
#elif defined( SX127X )
#include "ralf_sx127x.h"
#elif defined( LR20XX )
#include "ralf_lr20xx.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define RALF_RADIO_POINTER smtc_rac_get_rp( )->radio
#define RAL_RADIO_POINTER &( smtc_rac_get_rp( )->radio->ral )
/*!
 * \brief Default FSK synchronization word
 */
#define SMTC_RAC_FSK_DEFAULT_SYNC_WORD \
    {                                  \
        0xC1, 0x94, 0xC1               \
    }
/*!
 * \brief Default FSK configuration constants
 */
#define SMTC_RAC_FSK_WHITENING_SEED ( 0x01FF )
#define SMTC_RAC_FSK_CRC_SEED ( 0x1D0F )
#define SMTC_RAC_FSK_CRC_POLYNOMIAL ( 0x1021 )

/* Union of every IRQ either fsk_tx_callback or fsk_rx_callback might
 * want to see. The chip only fires IRQs relevant to its current mode
 * (RX_DONE doesn't fire while in TX, etc.), so a wider mask is
 * semantically identical to the narrow per-mode masks but avoids
 * having to re-issue SetDioIrqParams on every TX↔RX transition. With
 * the smart_set_dio_irq cache below, the union mask is set once at
 * the first transaction and skipped forever after. Saves ~30-50 us
 * per transition (one fewer SPI command). */
#define SMTC_RAC_FSK_IRQ_UNION_MASK \
    ( RAL_IRQ_TX_DONE | RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | \
      RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// Note: FSK contexts are now stored in the unified smtc_rac_context array in smtc_rac.c

// Default FSK sync word
static const uint8_t default_fsk_sync_word[] = SMTC_RAC_FSK_DEFAULT_SYNC_WORD;

/*
 * --- Chip-state cache (TX/RX setup short-circuit) -----------------------------
 *
 * Most field-by-field SPI commands issued from {tx,rx}_callback are
 * redundant under the typical "single PHY, RX continuous, occasional
 * TX" workload — every successful packet (RX_DONE or TX_DONE) leads
 * the planner to free the task and re-launch a new one with the same
 * params. ralf_setup_gfsk re-issues 9 commands. Each command costs at
 * least one BUSY-wait quantum (~100 us on Zephyr tickless), so the
 * full rebuild adds ~1 ms host-side latency on every re-arm even when
 * nothing changed.
 *
 * Cache the last-programmed params here and skip setters whose
 * arguments haven't changed since the last call. Same chip, same
 * planner thread → no concurrency. If the host re-issues a setup that
 * actually IS different (e.g. PHY change, TX↔RX with different
 * pld_len_in_bytes), the differing field still goes out.
 *
 * Invalidate by calling smtc_rac_fsk_chip_cache_invalidate() — done
 * automatically on PHY change via smtc_rac_radio_fsk_params_t writes
 * isn't possible here without coupling, so external code that mutates
 * the chip outside of this module (e.g. a direct ral_* poke from the
 * BSP) must call the invalidate hook itself.
 */
struct fsk_chip_cache {
    bool                                    valid;
    bool                                    irq_mask_valid;
    /* Last gfsk params we programmed. */
    uint32_t                                rf_freq_in_hz;
    int8_t                                  output_pwr_in_dbm;
    bool                                    pkt_type_set;        /* set_pkt_type(GFSK) issued */
    bool                                    stop_timer_set;
    bool                                    timer_stop_on_preamble;
    ral_gfsk_mod_params_t                   mod_params;
    ral_gfsk_pkt_params_t                   pkt_params;
    bool                                    crc_params_valid;
    uint32_t                                crc_seed;
    uint32_t                                crc_polynomial;
    uint8_t                                 sync_word[8];
    uint8_t                                 sync_word_len_bytes;
    bool                                    whitening_valid;
    uint16_t                                whitening_seed;
    /* Last DIO IRQ mask. */
    ral_irq_t                               irq_mask;
};

static struct fsk_chip_cache g_fsk_cache;

/* Public: drop the cache so the next setup goes back through the full
 * path. Call this after anything that bypasses the rac_fsk callbacks
 * to touch the chip directly (BSP pokes, direct ral_set_*). Today the
 * app's apply_phy_preset path goes through smtc_rac_fsk_set_*_params
 * which then re-runs ralf_setup_gfsk via the callback, so the cache
 * naturally re-validates with the new fields without an explicit
 * invalidate. */
void smtc_rac_fsk_chip_cache_invalidate( void )
{
    memset( &g_fsk_cache, 0, sizeof( g_fsk_cache ) );
}

/* Replacement for ralf_setup_gfsk that compares against g_fsk_cache
 * and only issues SPI commands for fields that actually changed.
 * Returns RAL_STATUS_OK if all (issued) commands succeeded. */
static ral_status_t smart_setup_gfsk( const ralf_t* radio,
                                      const ralf_params_gfsk_t* params )
{
    ral_status_t status;

    /* StopTimerOnPreamble: ralf_setup_gfsk always calls with false. */
    if( !g_fsk_cache.valid || !g_fsk_cache.stop_timer_set ||
        g_fsk_cache.timer_stop_on_preamble != false )
    {
        status = ral_stop_timer_on_preamble( &radio->ral, false );
        if( status != RAL_STATUS_OK ) return status;
        g_fsk_cache.stop_timer_set = true;
        g_fsk_cache.timer_stop_on_preamble = false;
    }

    if( !g_fsk_cache.valid || !g_fsk_cache.pkt_type_set )
    {
        status = ral_set_pkt_type( &radio->ral, RAL_PKT_TYPE_GFSK );
        if( status != RAL_STATUS_OK ) return status;
        g_fsk_cache.pkt_type_set = true;
    }

    if( !g_fsk_cache.valid || g_fsk_cache.rf_freq_in_hz != params->rf_freq_in_hz )
    {
        status = ral_set_rf_freq( &radio->ral, params->rf_freq_in_hz );
        if( status != RAL_STATUS_OK ) return status;
        g_fsk_cache.rf_freq_in_hz = params->rf_freq_in_hz;
    }

    /* set_tx_cfg programs PA + tx params; cheap to skip on RX re-arm
     * but the upstream ralf_setup_gfsk also runs it for RX setups, so
     * we keep the behaviour identical. Comparison is on power+freq;
     * BSP-derived PA params are deterministic from those. */
    if( !g_fsk_cache.valid ||
        g_fsk_cache.output_pwr_in_dbm != params->output_pwr_in_dbm ||
        g_fsk_cache.rf_freq_in_hz != params->rf_freq_in_hz )
    {
        status = ral_set_tx_cfg( &radio->ral, params->output_pwr_in_dbm, params->rf_freq_in_hz );
        if( status != RAL_STATUS_OK ) return status;
        g_fsk_cache.output_pwr_in_dbm = params->output_pwr_in_dbm;
    }

    if( !g_fsk_cache.valid ||
        memcmp( &g_fsk_cache.mod_params, &params->mod_params, sizeof( ral_gfsk_mod_params_t ) ) != 0 )
    {
        status = ral_set_gfsk_mod_params( &radio->ral, &params->mod_params );
        if( status != RAL_STATUS_OK ) return status;
        g_fsk_cache.mod_params = params->mod_params;
    }

    if( !g_fsk_cache.valid ||
        memcmp( &g_fsk_cache.pkt_params, &params->pkt_params, sizeof( ral_gfsk_pkt_params_t ) ) != 0 )
    {
        status = ral_set_gfsk_pkt_params( &radio->ral, &params->pkt_params );
        if( status != RAL_STATUS_OK ) return status;
        g_fsk_cache.pkt_params = params->pkt_params;
    }

    if( params->pkt_params.crc_type != RAL_GFSK_CRC_OFF )
    {
        if( !g_fsk_cache.valid || !g_fsk_cache.crc_params_valid ||
            g_fsk_cache.crc_seed != params->crc_seed ||
            g_fsk_cache.crc_polynomial != params->crc_polynomial )
        {
            status = ral_set_gfsk_crc_params( &radio->ral, params->crc_seed, params->crc_polynomial );
            if( status != RAL_STATUS_OK ) return status;
            g_fsk_cache.crc_seed = params->crc_seed;
            g_fsk_cache.crc_polynomial = params->crc_polynomial;
            g_fsk_cache.crc_params_valid = true;
        }
    }

    {
        uint8_t sync_len_bytes = ( params->pkt_params.sync_word_len_in_bits + 7 ) / 8;
        if( sync_len_bytes > sizeof( g_fsk_cache.sync_word ) )
        {
            sync_len_bytes = sizeof( g_fsk_cache.sync_word );
        }
        if( !g_fsk_cache.valid ||
            g_fsk_cache.sync_word_len_bytes != sync_len_bytes ||
            memcmp( g_fsk_cache.sync_word, params->sync_word, sync_len_bytes ) != 0 )
        {
            status = ral_set_gfsk_sync_word( &radio->ral, params->sync_word, sync_len_bytes );
            if( status != RAL_STATUS_OK ) return status;
            memcpy( g_fsk_cache.sync_word, params->sync_word, sync_len_bytes );
            g_fsk_cache.sync_word_len_bytes = sync_len_bytes;
        }
    }

    if( params->pkt_params.dc_free != RAL_GFSK_DC_FREE_OFF )
    {
        if( !g_fsk_cache.valid || !g_fsk_cache.whitening_valid ||
            g_fsk_cache.whitening_seed != params->whitening_seed )
        {
            status = ral_set_gfsk_whitening_seed( &radio->ral, params->whitening_seed );
            if( status != RAL_STATUS_OK ) return status;
            g_fsk_cache.whitening_seed = params->whitening_seed;
            g_fsk_cache.whitening_valid = true;
        }
    }

    g_fsk_cache.valid = true;
    return RAL_STATUS_OK;
}

/* Same idea for the DIO IRQ mask — TX and RX use different masks. */
static ral_status_t smart_set_dio_irq( const ralf_t* radio, ral_irq_t mask )
{
    if( g_fsk_cache.irq_mask_valid && g_fsk_cache.irq_mask == mask )
    {
        return RAL_STATUS_OK;
    }
    ral_status_t status = ral_set_dio_irq_params( &radio->ral, mask );
    if( status != RAL_STATUS_OK ) return status;
    g_fsk_cache.irq_mask = mask;
    g_fsk_cache.irq_mask_valid = true;
    return RAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rp_radio_params_t prepare_radio_params_for_fsk( smtc_rac_context_t* rac_config );
static void              smtc_rac_fsk_tx_callback( void* rp_void );
static void              smtc_rac_fsk_rx_callback( void* rp_void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

// Removed: smtc_rac_get_fsk_context() - now use smtc_rac_get_context() instead

smtc_rac_return_code_t smtc_rac_fsk( uint8_t radio_access_id )
{
    /**
     * @brief [TODO] , a function will be introduced to check all input parameters inside `smtc_rac_context_t`.
     *
     * This function will validate the compatibility of radio parameters and return in case of any incompatible
     * configuration.
     */

    smtc_modem_hal_protect_api_call( );
    smtc_rac_context_t* rac_config = smtc_rac_get_context( radio_access_id );

    RAC_LOG_CONFIG( "FSK: Starting %s for radio ID %d\n",
                    rac_config->radio_params.fsk.is_tx ? "transmission" : "reception", radio_access_id );

    // Set modulation type to FSK
    rac_config->modulation_type = SMTC_RAC_MODULATION_FSK;

    rp_radio_params_t rp_radio_params = prepare_radio_params_for_fsk( rac_config );

    const rp_task_t rp_task = {
        .hook_id = radio_access_id,
        .type    = ( rac_config->radio_params.fsk.is_tx == true ) ? RP_TASK_TYPE_TX_FSK : RP_TASK_TYPE_RX_FSK,
        .state = ( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ) ? RP_TASK_STATE_SCHEDULE
                                                                                               : RP_TASK_STATE_ASAP,

        .schedule_task_low_priority = false,
        .duration_time_ms           = ral_get_gfsk_time_on_air_in_ms(
            RAL_RADIO_POINTER,
            ( rac_config->radio_params.fsk.is_tx == true ) ? &( rp_radio_params.tx.gfsk.pkt_params )
                                                                     : &( rp_radio_params.rx.gfsk.pkt_params ),
            ( rac_config->radio_params.fsk.is_tx == true ) ? &( rp_radio_params.tx.gfsk.mod_params )
                                                                     : &( rp_radio_params.rx.gfsk.mod_params ) ),
        .start_time_ms = rac_config->scheduler_config.start_time_ms,
        .launch_task_callbacks =
            ( rac_config->radio_params.fsk.is_tx == true ) ? smtc_rac_fsk_tx_callback : smtc_rac_fsk_rx_callback,
    };

    if( rac_config->radio_params.fsk.is_tx )
    {
        RAC_LOG_TX( "FSK: Configuring TX - Freq:%lu Hz, Power:%d dBm, Bitrate:%lu bps, Payload:%d bytes\n",
                    rp_radio_params.tx.gfsk.rf_freq_in_hz, rp_radio_params.tx.gfsk.output_pwr_in_dbm,
                    rp_radio_params.tx.gfsk.mod_params.br_in_bps, rac_config->radio_params.fsk.tx_size );
    }
    else
    {
        RAC_LOG_RX( "FSK: Configuring RX - Freq:%lu Hz, Bitrate:%lu bps\n", rp_radio_params.rx.gfsk.rf_freq_in_hz,
                    rp_radio_params.rx.gfsk.mod_params.br_in_bps );
    }

    if( rp_task_enqueue( smtc_rac_get_rp( ), &rp_task,
                         ( rac_config->radio_params.fsk.is_tx )
                             ? rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer
                             : rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer,
                         ( rac_config->radio_params.fsk.is_tx ) ? rac_config->radio_params.fsk.tx_size
                                                                : rac_config->radio_params.fsk.max_rx_size,
                         &rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        RAC_LOG_ERROR( "FSK: Error enqueueing task for radio access ID %d\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    RAC_LOG_DEBUG( "FSK: Task successfully enqueued for radio ID %d\n", radio_access_id );
    smtc_modem_hal_unprotect_api_call( );
    return SMTC_RAC_SUCCESS;
}

smtc_rac_return_code_t smtc_rac_start_radio_transaction( uint8_t radio_access_id )
{
    smtc_modem_hal_protect_api_call( );
    smtc_rac_context_t*    rac_config = smtc_rac_get_context( radio_access_id );
    smtc_rac_return_code_t result;

    // Route to appropriate function based on modulation type
    switch( rac_config->modulation_type )
    {
    case SMTC_RAC_MODULATION_LORA:
        smtc_modem_hal_unprotect_api_call( );
        result = smtc_rac_lora( radio_access_id );
        break;

    case SMTC_RAC_MODULATION_FSK:
        smtc_modem_hal_unprotect_api_call( );
        result = smtc_rac_fsk( radio_access_id );
        break;

    case SMTC_RAC_MODULATION_LRFHSS:
        smtc_modem_hal_unprotect_api_call( );
        result = smtc_rac_lrfhss( radio_access_id );
        break;

    default:
        RAC_LOG_ERROR( "Invalid modulation type %d\n", rac_config->modulation_type );
        smtc_modem_hal_unprotect_api_call( );
        result = SMTC_RAC_INVALID_PARAMETER;
        break;
    }

    return result;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static rp_radio_params_t prepare_radio_params_for_fsk( smtc_rac_context_t* rac_config )
{
    ralf_params_gfsk_t gfsk_param = { 0 };
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    // Basic radio parameters
    gfsk_param.rf_freq_in_hz     = rac_config->radio_params.fsk.frequency_in_hz;
    gfsk_param.output_pwr_in_dbm = rac_config->radio_params.fsk.tx_power_in_dbm;

    // Use provided sync word or default
    gfsk_param.sync_word = ( rac_config->radio_params.fsk.sync_word != NULL ) ? rac_config->radio_params.fsk.sync_word
                                                                              : default_fsk_sync_word;

    // Advanced parameters with defaults if not set
    gfsk_param.whitening_seed = ( rac_config->radio_params.fsk.whitening_seed != 0 )
                                    ? rac_config->radio_params.fsk.whitening_seed
                                    : SMTC_RAC_FSK_WHITENING_SEED;
    gfsk_param.crc_seed =
        ( rac_config->radio_params.fsk.crc_seed != 0 ) ? rac_config->radio_params.fsk.crc_seed : SMTC_RAC_FSK_CRC_SEED;
    gfsk_param.crc_polynomial = ( rac_config->radio_params.fsk.crc_polynomial != 0 )
                                    ? rac_config->radio_params.fsk.crc_polynomial
                                    : SMTC_RAC_FSK_CRC_POLYNOMIAL;

    // Packet parameters
    gfsk_param.pkt_params.header_type           = rac_config->radio_params.fsk.header_type;
    gfsk_param.pkt_params.pld_len_in_bytes      = rac_config->radio_params.fsk.tx_size;
    gfsk_param.pkt_params.preamble_len_in_bits  = rac_config->radio_params.fsk.preamble_len_in_bits;
    gfsk_param.pkt_params.preamble_detector     = rac_config->radio_params.fsk.preamble_detector;
    gfsk_param.pkt_params.sync_word_len_in_bits = rac_config->radio_params.fsk.sync_word_len_in_bits;
    gfsk_param.pkt_params.crc_type              = rac_config->radio_params.fsk.crc_type;
    gfsk_param.pkt_params.dc_free               = rac_config->radio_params.fsk.dc_free;

    // Modulation parameters
    gfsk_param.mod_params.br_in_bps    = rac_config->radio_params.fsk.br_in_bps;
    gfsk_param.mod_params.fdev_in_hz   = rac_config->radio_params.fsk.fdev_in_hz;
    gfsk_param.mod_params.bw_dsb_in_hz = rac_config->radio_params.fsk.bw_dsb_in_hz;
    gfsk_param.mod_params.pulse_shape  = rac_config->radio_params.fsk.pulse_shape;

    // Configure radio parameters
    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.pkt_type          = RAL_PKT_TYPE_GFSK;

    if( rac_config->radio_params.fsk.is_tx == true )

    {
        gfsk_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.fsk.tx_size;
        rp_radio_params.tx.gfsk                = gfsk_param;
    }
    else
    {
        gfsk_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.fsk.max_rx_size;
        rp_radio_params.rx.gfsk                = gfsk_param;
        rp_radio_params.rx.timeout_in_ms       = rac_config->radio_params.fsk.rx_timeout_ms;
    }

    return rp_radio_params;
}

static void smtc_rac_fsk_tx_callback( void* rp_void )
{
    radio_planner_t*    rp           = ( radio_planner_t* ) rp_void;
    uint8_t             id           = rp->radio_task_id;
    rp_radio_params_t*  radio_params = &rp->radio_params[id];
    smtc_rac_context_t* rac_config   = smtc_rac_get_context( id );
    if( rac_config->lbt_context.lbt_enabled )
    {
        smtc_rac_lbt_listen_channel( id, radio_params->tx.lora.rf_freq_in_hz,
                                     rac_config->lbt_context.listen_duration_ms, rac_config->lbt_context.threshold_dbm,
                                     rac_config->lbt_context.bandwidth_hz );

        if( rp->status[id] == RP_STATUS_LBT_BUSY_CHANNEL )
        {
            RAC_LOG_INFO( "LBT: Channel is busy, aborting transmission" );
            rp_radio_irq_callback( rp );

            return;
        }
        else if( rp->status[id] == RP_STATUS_LBT_FREE_CHANNEL )
        {
            RAC_LOG_INFO( "LBT: Channel is free, continuing transmission" );
        }
    }
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smart_setup_gfsk( rp->radio, &rp->radio_params[id].tx.gfsk ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smart_set_dio_irq( rp->radio, SMTC_RAC_FSK_IRQ_UNION_MASK ) == RAL_STATUS_OK );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_buffer_size[id] ) == RAL_STATUS_OK );

    // Wait the exact expected time (ie target - tcxo startup delay)
    smtc_rac_context_t* rac_context = smtc_rac_get_context( id );
    if( rac_context->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        rac_context->scheduler_config.callback_pre_radio_transaction( );
    }

    rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) > 0 )
    {
        rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    }

    // At this time only tcxo startup delay is remaining
    if( rac_config->lbt_context.lbt_enabled == false )
    {
        smtc_modem_hal_start_radio_tcxo( );
    }
    smtc_modem_hal_set_ant_switch( true );
    if( rac_config->cw_context.cw_enabled )
    {
        rp_disable_failsafe( rp, true );
        if( rac_config->cw_context.infinite_preamble )
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx_infinite_preamble( &( rp->radio->ral ) ) == RAL_STATUS_OK );
        }
        else
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx_cw( &( rp->radio->ral ) ) == RAL_STATUS_OK );
        }
    }
    else
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    }
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_TX( "FSK Tx callback - Freq:%lu Hz, Power:%d dBm, Bitrate:%lu bps, Fdev:%lu Hz, BW:%lu Hz, length:%u\n",
                radio_params->tx.gfsk.rf_freq_in_hz, radio_params->tx.gfsk.output_pwr_in_dbm,
                radio_params->tx.gfsk.mod_params.br_in_bps, radio_params->tx.gfsk.mod_params.fdev_in_hz,
                radio_params->tx.gfsk.mod_params.bw_dsb_in_hz, rp->payload_buffer_size[id] );
}

static void smtc_rac_fsk_rx_callback( void* rp_void )
{
    radio_planner_t*    rp           = ( radio_planner_t* ) rp_void;
    uint8_t             id           = rp->radio_task_id;
    rp_radio_params_t*  radio_params = &rp->radio_params[id];
    smtc_rac_context_t* rac_config   = smtc_rac_get_context( id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smart_setup_gfsk( rp->radio, &radio_params->rx.gfsk ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smart_set_dio_irq( rp->radio, SMTC_RAC_FSK_IRQ_UNION_MASK ) == RAL_STATUS_OK );

    // Wait the exact expected time (ie target - tcxo startup delay)
    smtc_rac_context_t* rac_context = smtc_rac_get_context( id );
    if( rac_context->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        rac_context->scheduler_config.callback_pre_radio_transaction( );
    }

    rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) > 0 )
    {
        rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    }

    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), radio_params->rx.timeout_in_ms ) ==
                                     RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_RX( "FSK Rx callback - Freq:%lu Hz, Bitrate:%lu bps, Fdev:%lu Hz, BW:%lu Hz\n",
                radio_params->rx.gfsk.rf_freq_in_hz, radio_params->rx.gfsk.mod_params.br_in_bps,
                radio_params->rx.gfsk.mod_params.fdev_in_hz, radio_params->rx.gfsk.mod_params.bw_dsb_in_hz );
}

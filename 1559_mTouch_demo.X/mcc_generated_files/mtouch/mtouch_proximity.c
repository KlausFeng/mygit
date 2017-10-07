/*
    MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:

    You may use this software, and any derivatives created by any person or
    entity by or on your behalf, exclusively with Microchip's products.
    Microchip and its subsidiaries ("Microchip"), and its licensors, retain all
    ownership and intellectual property rights in the accompanying software and
    in all derivatives hereto.

    This software and any accompanying information is for suggestion only. It
    does not modify Microchip's standard warranty for its products.  You agree
    that you are solely responsible for testing the software and determining
    its suitability.  Microchip has no obligation to modify, test, certify, or
    support the software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP'S
    PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
    (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
    INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
    ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
    FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL
    LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
    THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
    THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
    THESE TERMS.
*/
#include <xc.h>
#include <stdint.h>
#include <string.h>

#include "mtouch.h"
#include "mtouch_proximity.h"

enum mtouch_prox_state
{
    MTOUCH_PROXIMITY_STATE_initializing = 0,
    MTOUCH_PROXIMITY_STATE_notActivated = 1,
    MTOUCH_PROXIMITY_STATE_activated    = 2
};
    
/* 
 * =======================================================================
 * Proximity Type Structure
 * =======================================================================
 */
    typedef struct
    {
        const uint8_t                       name;
        const enum mtouch_sensor_names      sensor;
              enum mtouch_prox_state        state;
              mtouch_prox_reading_t         reading;
              mtouch_prox_baseline_t        baseline;
              mtouch_prox_deviation_t       integratedDeviation;
              mtouch_prox_statecounter_t    counter;
              mtouch_prox_baselinecounter_t baseline_count;
              mtouch_prox_deviation_t       threshold;
              mtouch_prox_scaling_t         scaling;
    } mtouch_proximity_t;
 
    const  mtouch_proximity_t mtouch_proximity_init[MTOUCH_PROXIMITY] =
    {
        /* Proximity0 */
        {   Proximity0, 
            Sensor_AN14,
            MTOUCH_PROXIMITY_STATE_initializing,0,0,0,0,0, /* non-const variables */
            (mtouch_prox_deviation_t)MTOUCH_PROXIMITY_THRESHOLD_Proximity0, /* threshold */
            (uint8_t)MTOUCH_PROXIMITY_SCALING_Proximity0 /* scaling */
        }
    };
    
    static mtouch_proximity_t mtouch_proximity[MTOUCH_PROXIMITY];

/*
 * =======================================================================
 *  Local Functions
 * =======================================================================
 */
static void                     Proximity_Service               (enum mtouch_proximity_names name);
static void                     Proximity_Deviation_Update      (enum mtouch_proximity_names prox);
static void                     Proximity_Reading_Update        (mtouch_proximity_t* prox);
static void                     Proximity_Baseline_Initialize   (mtouch_proximity_t* prox);
static void                     Proximity_Baseline_Update       (mtouch_proximity_t* prox);
static mtouch_prox_reading_t    Proximity_Baseline_Get_helper   (enum mtouch_proximity_names prox);
static void                     Proximity_Tick_helper           (mtouch_proximity_t* prox);
static void                     Proximity_DefaultCallback       (enum mtouch_proximity_names prox);
static void                     Proximity_State_Initializing    (mtouch_proximity_t* prox);
static void                     Proximity_State_NotActivated    (mtouch_proximity_t* prox);
static void                     Proximity_State_Activated       (mtouch_proximity_t* prox);

/*
 * =======================================================================
 *  Callback Function Pointers
 * =======================================================================
 */
static void (*callback_activated)   (enum mtouch_proximity_names) = Proximity_DefaultCallback;
static void (*callback_notActivated)(enum mtouch_proximity_names) = Proximity_DefaultCallback;


/*
 * =======================================================================
 * Proximity Statemachine
 * =======================================================================
 */
typedef void (*proximity_statemachine_state_t)(mtouch_proximity_t*);
proximity_statemachine_state_t Proximity_StateMachine[] = 
{
    Proximity_State_Initializing,
    Proximity_State_NotActivated,
    Proximity_State_Activated
};
#define PROXIMITY_STATEMACHINE_COUNT (uint8_t)(sizeof(Proximity_StateMachine)/sizeof(proximity_statemachine_state_t))

/*
 * =======================================================================
 *  MTOUCH_Proximity_Initialize
 * =======================================================================
 */
void MTOUCH_Proximity_Initialize(enum mtouch_proximity_names name)
{
    mtouch_proximity_t* prox = &mtouch_proximity[name];
    
    prox->state           = MTOUCH_PROXIMITY_STATE_initializing;
    prox->baseline        = (mtouch_prox_baseline_t)0;
    prox->counter         = (mtouch_prox_statecounter_t)0;
    prox->baseline_count  = (mtouch_prox_baselinecounter_t)0;
    
}

void MTOUCH_proximity_Recalibrate(void)
{
    uint8_t i;
    for (i = 0; i < MTOUCH_PROXIMITY; i++)
    {
        MTOUCH_Proximity_Initialize(i);
    }
}

void MTOUCH_Proximity_InitializeAll(void)
{
    memcpy(mtouch_proximity,mtouch_proximity_init,sizeof(mtouch_proximity_init));
    
    MTOUCH_Proximity_Initialize(Proximity0);
}


void MTOUCH_Proximity_ServiceAll(void)
{
    Proximity_Service(Proximity0);
}

/*
 * =======================================================================
 * Proximity Service
 * =======================================================================
 */
static void Proximity_Service(enum mtouch_proximity_names name)
{
    mtouch_proximity_t* prox = &mtouch_proximity[name];
        
    if (MTOUCH_Sensor_wasSampled(prox->sensor) && MTOUCH_Sensor_isCalibrated(prox->sensor))
    {
        Proximity_Reading_Update(prox);
        Proximity_Deviation_Update(name);

        if (prox->state >= PROXIMITY_STATEMACHINE_COUNT)
        {
            MTOUCH_Proximity_Initialize(prox->name);
        }
        Proximity_StateMachine[prox->state](prox);
    }
}
static void Proximity_State_Initializing(mtouch_proximity_t* prox)
{
    /* Initialize the baseline. */
    if ((prox->counter) <= (mtouch_prox_statecounter_t)MTOUCH_PROXIMITY_BASELINE_INIT/2)
    { 
        Proximity_Baseline_Initialize(prox); /* Rough resolution */
    } else { 
        Proximity_Baseline_Update(prox); /* Fine resolution */
    }
    
    /* Exit after a fixed number of baseline updates */
    (prox->counter)++;
    if ((prox->counter) >= (mtouch_prox_statecounter_t)MTOUCH_PROXIMITY_BASELINE_INIT)
    {
        prox->state   = MTOUCH_PROXIMITY_STATE_notActivated;
        prox->counter = (mtouch_prox_statecounter_t)0;
    }
} 
static void Proximity_State_NotActivated(mtouch_proximity_t* prox)
{
    /* Negative Capacitance check */
    int32_t deviation = (int32_t)((uint32_t)(MTOUCH_Proximity_Reading_Get(prox->name)) - (uint32_t)(MTOUCH_Proximity_Baseline_Get(prox->name)));
    if (deviation < (int32_t)MTOUCH_PROXIMITY_NEGATIVETHRESHOLD)
    {
        (prox->counter)++;
        if ((prox->counter) > (mtouch_prox_statecounter_t)MTOUCH_PROXIMITY_NEGATIVEDEVIATION)
        {
            prox->counter = (mtouch_prox_statecounter_t)0;
            MTOUCH_Proximity_Initialize(prox->name);
        }
    }
    /* Threshold check */
    else if ((prox->integratedDeviation) > (prox->threshold))
    {
        prox->state   = MTOUCH_PROXIMITY_STATE_activated;
        prox->counter = (mtouch_prox_statecounter_t)0;
        callback_activated(prox->name);
    }
    else
    {
        /* Delta is positive, but not past the threshold. */
        prox->counter = (mtouch_prox_statecounter_t)0;
    }
    
    /* Baseline Update check */
    (prox->baseline_count)++;
    if ((prox->baseline_count) > MTOUCH_PROXIMITY_BASELINE_RATE)
    {
        prox->baseline_count = (mtouch_prox_baselinecounter_t)0;
        Proximity_Baseline_Update(prox);
    }
}
static void Proximity_State_Activated(mtouch_proximity_t* prox)
{
    /* Timeout check */
    if ((prox->counter) >= MTOUCH_PROXIMITY_ACTIVITYTIMEOUT)
    {
        MTOUCH_Proximity_Initialize(prox->name);
        callback_notActivated(prox->name);
    }
    
    /* Threshold check */
    else if ((prox->integratedDeviation) < (mtouch_prox_deviation_t)((prox->threshold) >> 1))
    {
        prox->state   = MTOUCH_PROXIMITY_STATE_notActivated;
        prox->counter = (mtouch_prox_statecounter_t)0;
        callback_notActivated(prox->name);
    }
}

/*
 * =======================================================================
 *  MTOUCH_Proximity_Tick
 * =======================================================================
 */
void MTOUCH_Proximity_Tick(void)
{
    Proximity_Tick_helper(&mtouch_proximity[0]);
}
static void Proximity_Tick_helper(mtouch_proximity_t* prox)
{
    /* Only pressed state counter is based on real time */
    if ((prox->state) == MTOUCH_PROXIMITY_STATE_activated)
    {
        (prox->counter)++;
        if (prox->counter == (mtouch_prox_statecounter_t)0)
        {
            prox->counter = (mtouch_prox_statecounter_t)0xFFFF;
        }
    }
}


/*
 * =======================================================================
 * Proximity State and Deviation
 * =======================================================================
 */

mtouch_prox_threshold_t MTOUCH_Proximity_Threshold_Get(enum mtouch_proximity_names name)
{
    return mtouch_proximity[name].threshold;
}

void MTOUCH_Proximity_Threshold_Set(enum mtouch_proximity_names name,mtouch_prox_threshold_t threshold)
{
    if(threshold >= MTOUCH_PROXIMITY_THRESHOLD_MIN && threshold <= MTOUCH_PROXIMITY_THRESHOLD_MAX)
        mtouch_proximity[name].threshold = threshold;
}

mtouch_prox_scaling_t MTOUCH_Proximity_Scaling_Get(enum mtouch_proximity_names name)
{
    return mtouch_proximity[name].scaling;
}

void MTOUCH_Proximity_Scaling_Set(enum mtouch_proximity_names name,mtouch_prox_scaling_t scaling)
{
    if(scaling <= MTOUCH_PROXIMITY_SCALING_MAX)
        mtouch_proximity[name].scaling = scaling;
}

bool MTOUCH_Proximity_isActivated(enum mtouch_proximity_names name)
{
    return (mtouch_proximity[name].state == MTOUCH_PROXIMITY_STATE_activated) ? true : false;
}
mtouch_prox_deviation_t MTOUCH_Proximity_Deviation_Get(enum mtouch_proximity_names name) /* Global */
{
    return mtouch_proximity[name].integratedDeviation;
}

static void Proximity_Deviation_Update(enum mtouch_proximity_names name)
{
    int32_t deviation;

    deviation = (int32_t)((uint32_t)(MTOUCH_Proximity_Reading_Get(name)) - (uint32_t)(MTOUCH_Proximity_Baseline_Get(name)));

    /* Delta scaling */
    deviation = deviation >> (mtouch_proximity[name].scaling); /* XC8 compiler will extend sign bit of signed values */

    /* Bounds checking */
    if (deviation > MTOUCH_PROXIMITY_DEVIATION_MAX)
    {
        deviation = MTOUCH_PROXIMITY_DEVIATION_MAX;
    }
    if (deviation < MTOUCH_PROXIMITY_DEVIATION_MIN)
    {
        deviation = MTOUCH_PROXIMITY_DEVIATION_MIN;
    }

    if(deviation < 0)
    {
        mtouch_proximity[name].integratedDeviation = 0;
    }
    else if((mtouch_prox_deviation_t)deviation < (mtouch_prox_deviation_t)(mtouch_proximity[name].threshold << 3)) 
    {
        /* decrement the accumulation of deviation when the deviation is less than 12.5% of the threshold */
        if(mtouch_proximity[name].integratedDeviation >0 )
            mtouch_proximity[name].integratedDeviation--;
    }
    else
    {
        mtouch_proximity[name].integratedDeviation -= mtouch_proximity[name].integratedDeviation >> MTOUCH_PROXIMITY_DEVIATION_GAIN;
        if(MTOUCH_PROXIMITY_DEVIATION_MAX - mtouch_proximity[name].integratedDeviation > deviation)
            mtouch_proximity[name].integratedDeviation += deviation;
        else
            mtouch_proximity[name].integratedDeviation = MTOUCH_PROXIMITY_DEVIATION_MAX;
    }
}
 
/*
 * =======================================================================
 * Proximity Reading and Baseline
 * =======================================================================
 */
mtouch_prox_reading_t MTOUCH_Proximity_Reading_Get(enum mtouch_proximity_names name) /* Global */
{
    if (name < MTOUCH_PROXIMITY)
        return mtouch_proximity[name].reading;
    else
        return 0;
}

static void Proximity_Reading_Update(mtouch_proximity_t* prox)
{
    if (INTCONbits.GIE == (uint8_t)1)
    {
        INTCONbits.GIE = (uint8_t)0;
        prox->reading -= prox->reading >> MTOUCH_PROXIMITY_READING_GAIN;
        prox->reading += MTOUCH_Sensor_RawSample_Get(prox->sensor);
        INTCONbits.GIE = (uint8_t)1;
    }
    else
    {
        prox->reading -= prox->reading >> MTOUCH_PROXIMITY_READING_GAIN;
        prox->reading += MTOUCH_Sensor_RawSample_Get(prox->sensor);
    }
}

static void Proximity_Baseline_Initialize(mtouch_proximity_t* prox)
{
    prox->baseline   = MTOUCH_Sensor_RawSample_Get(prox->sensor);
    prox->baseline <<= (MTOUCH_PROXIMITY_BASELINE_GAIN + MTOUCH_PROXIMITY_READING_GAIN);
}

static void Proximity_Baseline_Update(mtouch_proximity_t* prox)
{
    if (INTCONbits.GIE == (uint8_t)1)
    {
        INTCONbits.GIE = (uint8_t)0;
        prox->baseline -= (prox->baseline) >> MTOUCH_PROXIMITY_BASELINE_GAIN;
        prox->baseline += prox->reading;
        INTCONbits.GIE = (uint8_t)1;
    }
    else
    {
        prox->baseline -= (prox->baseline) >> MTOUCH_PROXIMITY_BASELINE_GAIN;
        prox->baseline += prox->reading;
    }
}

mtouch_prox_reading_t MTOUCH_Proximity_Baseline_Get(enum mtouch_proximity_names name) /* Global */
{
    if (name < MTOUCH_PROXIMITY)
        return Proximity_Baseline_Get_helper(name);
    else
        return 0;

}

static mtouch_prox_reading_t Proximity_Baseline_Get_helper(enum mtouch_proximity_names name)
{
    return (mtouch_prox_reading_t)(mtouch_proximity[name].baseline >> MTOUCH_PROXIMITY_BASELINE_GAIN);
}

/*
 * =======================================================================
 * Proximity Press/Release Callback Setter Functions
 * =======================================================================
 */
static void Proximity_DefaultCallback(enum mtouch_proximity_names name) { }
void MTOUCH_Proximity_SetActivatedCallback(void (*callback)(enum mtouch_proximity_names))
{
    callback_activated = callback;
}
void MTOUCH_Proximity_SetNotActivatedCallback(void (*callback)(enum mtouch_proximity_names))
{
    callback_notActivated = callback;
}


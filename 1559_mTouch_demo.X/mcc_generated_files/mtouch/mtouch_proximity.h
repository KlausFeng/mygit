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
#ifndef MTOUCH_PROXIMITY_H
#define MTOUCH_PROXIMITY_H
    
    #include <stdint.h>
    #include <stdbool.h>
    #include "mtouch.h"
    
/*
 * =======================================================================
 * Application / Configuration Settings
 * =======================================================================
 */
    #define MTOUCH_PROXIMITY 1
    
    enum mtouch_proximity_names
    {
        Proximity0 = 0
    };
    
    #define MTOUCH_PROXIMITY_THRESHOLD_Proximity0 120
    
    #define MTOUCH_PROXIMITY_SCALING_Proximity0 0
  
/*
 * =======================================================================
 * Typedefs / Data Types
 * =======================================================================
 */
    typedef uint16_t mtouch_prox_statecounter_t;
    #define MTOUCH_PROXIMITY_STATECOUNTER_MIN (0)
    #define MTOUCH_PROXIMITY_STATECOUNTER_MAX (UINT16_MAX)
    #define MTOUCH_PROXIMITY_NEGATIVEDEVIATION ((mtouch_prox_statecounter_t)32)
    #define MTOUCH_PROXIMITY_ACTIVITYTIMEOUT ((mtouch_prox_statecounter_t)1000)
    
    typedef uint16_t mtouch_prox_baselinecounter_t;
    #define MTOUCH_PROXIMITY_BASECOUNTER_MIN (0)
    #define MTOUCH_PROXIMITY_BASECOUNTER_MAX (UINT16_MAX)
    
    typedef uint32_t mtouch_prox_reading_t;
    #define MTOUCH_PROXIMITY_READING_MIN ((mtouch_prox_reading_t)0)
    #define MTOUCH_PROXIMITY_READING_MAX ((mtouch_prox_reading_t)UINT32_MAX)
    #define MTOUCH_PROXIMITY_READING_GAIN   ((uint8_t)1)
    
    typedef uint32_t mtouch_prox_baseline_t;
    #define MTOUCH_PROXIMITY_BASELINE_MIN (0)
    #define MTOUCH_PROXIMITY_BASELINE_MAX (UINT32_MAX)
    #define MTOUCH_PROXIMITY_BASELINE_GAIN ((uint8_t)4)
    #define MTOUCH_PROXIMITY_BASELINE_INIT ((mtouch_prox_statecounter_t)16.0)
    #define MTOUCH_PROXIMITY_BASELINE_RATE ((mtouch_prox_baselinecounter_t)64)
    
    typedef int8_t mtouch_prox_deviation_t;
    #define MTOUCH_PROXIMITY_DEVIATION_MIN (mtouch_prox_deviation_t)(INT8_MIN)
    #define MTOUCH_PROXIMITY_DEVIATION_MAX (mtouch_prox_deviation_t)(INT8_MAX)
    #define MTOUCH_PROXIMITY_NEGATIVETHRESHOLD (mtouch_prox_deviation_t)-10
    #define MTOUCH_PROXIMITY_DEVIATION_GAIN ((uint8_t)2);

    typedef uint8_t mtouch_prox_threshold_t;
    #define MTOUCH_PROXIMITY_THRESHOLD_MIN ((mtouch_prox_threshold_t)(1))
    #define MTOUCH_PROXIMITY_THRESHOLD_MAX ((mtouch_prox_threshold_t)(127))

    typedef uint8_t mtouch_prox_scaling_t;
    #define MTOUCH_PROXIMITY_SCALING_MIN ((mtouch_prox_scaling_t)(0))
    #define MTOUCH_PROXIMITY_SCALING_MAX ((mtouch_prox_scaling_t)(8))

/*
 * =======================================================================
 * Global Functions
 * =======================================================================
 */
    void MTOUCH_Proximity_SetActivatedCallback   (void (*callback)(enum mtouch_proximity_names prox));
    void MTOUCH_Proximity_SetNotActivatedCallback(void (*callback)(enum mtouch_proximity_names prox));
    void                        MTOUCH_Proximity_Initialize     (enum mtouch_proximity_names prox);
    void                        MTOUCH_Proximity_InitializeAll  (void);
    void                        MTOUCH_Proximity_ServiceAll     (void);
    void                        MTOUCH_Proximity_Tick           (void);
    void                        MTOUCH_Proximity_Recalibrate    (void);

    mtouch_prox_deviation_t     MTOUCH_Proximity_Deviation_Get  (enum mtouch_proximity_names prox);
    mtouch_prox_reading_t       MTOUCH_Proximity_Reading_Get    (enum mtouch_proximity_names prox);
    mtouch_prox_reading_t       MTOUCH_Proximity_Baseline_Get   (enum mtouch_proximity_names prox);
    mtouch_prox_scaling_t       MTOUCH_Proximity_Scaling_Get    (enum mtouch_proximity_names prox);
    void                        MTOUCH_Proximity_Scaling_Set    (enum mtouch_proximity_names prox,mtouch_prox_scaling_t scaling);
    void                        MTOUCH_Proximity_Threshold_Set  (enum mtouch_proximity_names prox,mtouch_prox_threshold_t threshold);
    mtouch_prox_threshold_t     MTOUCH_Proximity_Threshold_Get  (enum mtouch_proximity_names prox);
    bool                        MTOUCH_Proximity_isActivated    (enum mtouch_proximity_names prox);

#endif // MTOUCH_PROXIMITY_H

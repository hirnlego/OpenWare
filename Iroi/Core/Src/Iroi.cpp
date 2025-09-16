#include <string.h>
#include <stdlib.h>
#include "device.h"
#include "Owl.h"
#include "errorhandlers.h"
#include "message.h"
#include "ProgramManager.h"
#include "OpenWareMidiControl.h"
#include "Pin.h"
#include "Storage.h"
#include "MidiMessage.h"
#include "cmsis_os.h"

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif

#define PATCH_RESET_COUNTER (500 / MAIN_LOOP_SLEEP_MS)

#define INLEVELGREEN PARAMETER_AF
#define MOD PARAMETER_AG

// GPIO
#define MOD_CV_BUTTON PUSHBUTTON
#define MOD_CV_GREEN GREEN_BUTTON
#define MOD_CV_RED RED_BUTTON
#define RANDOM_BUTTON BUTTON_1
#define RANDOM_MAP BUTTON_2
#define RANDOM_GATE BUTTON_3
#define SYNC_GATE BUTTON_4
#define INLEVELRED BUTTON_5
#define SHIFT_BUTTON BUTTON_6
#define IN_DETEC BUTTON_7

// ADC3
#define FILTER_VOL2 PARAMETER_A
#define RESO_VOL2 PARAMETER_B
#define DELAY_VOL2 PARAMETER_C
#define REVERB_VOL2 PARAMETER_D
#define REVERBCV PARAMETER_E
#define DELAYCV PARAMETER_F

// ADC1
#define RESONATORCV PARAMETER_G
#define MOD_LEVEL PARAMETER_H
#define MOD_SPEED PARAMETER_AA
#define FILTERCV PARAMETER_AB
#define INLEVELLEDGREEN PARAMETER_AC

// ADC1 muxed (pots)
#define FILTER_CUTOFF PARAMETER_BA
#define FILTER_RESONANCE PARAMETER_BB
#define RESONATOR_TUNE PARAMETER_BC
#define RESONATOR_FEEDBACK PARAMETER_BD
#define ECHO_DENSITY PARAMETER_BE
#define ECHO_REPEATS PARAMETER_BF
#define AMBIENCE_SPACETIME PARAMETER_BG
#define AMBIENCE_DECAY PARAMETER_BH

enum leds
{
    RANDOM_LED = 1,
    RANDOM_MAP_LED,
    SYNC_LED,
    INLEVELRED_LED,
    INLEVELGREEN_LED,
    MOD_LED,
    SHIFT_LED,
    MOD_CV_GREEN_LED,
    MOD_CV_RED_LED,
};

enum ConfigMode
{
    CONFIG_MODE_NONE,
    CONFIG_MODE_OPTIONS,
};

struct Configuration
{
    bool mod_attenuverters;
    bool cv_attenuverters;
    int revision;
};

Configuration configuration;

ConfigMode configMode = CONFIG_MODE_NONE;

bool needsConfiguration = false;

static bool randomButtonState = false;
static bool randomMapButtonState = false;
static bool shiftButtonState = false;
static bool modCvButtonState = false;
static uint16_t mux_values[NOF_MUX_VALUES] DMA_RAM = {};

Pin randomButton(RANDOM_BUTTON_GPIO_Port, RANDOM_BUTTON_Pin);
Pin randomMapButton(RND_MAP_BUTTON_GPIO_Port, RND_MAP_BUTTON_Pin);
Pin shiftButton(SHIFT_BUTTON_GPIO_Port, SHIFT_BUTTON_Pin);
Pin modCvButton(MOD_AMT_BUTTON_GPIO_Port, MOD_AMT_BUTTON_Pin);

// MUX binary counter digital output pins
Pin muxA(MUX_A_GPIO_Port, MUX_A_Pin);
Pin muxB(MUX_B_GPIO_Port, MUX_B_Pin);
Pin muxC(MUX_C_GPIO_Port, MUX_C_Pin);

void saveConfiguration()
{
    debugMessage("Saving configuration");
    uint32_t headerSize = sizeof(ResourceHeader);
    uint32_t dataSize = sizeof(configuration);
    uint8_t buffer[headerSize + dataSize];
    memset(buffer, 0, headerSize);
    memcpy(buffer + headerSize, &configuration, dataSize);
    const char *filename = "iroi.cfg";
    taskENTER_CRITICAL();
    storage.writeResource(filename, buffer, dataSize, FLASH_DEFAULT_FLAGS);
    taskEXIT_CRITICAL();
    debugMessage(filename, (int)dataSize);
}

void loadConfiguration()
{
    Resource *resource = storage.getResourceByName("iroi.cfg");
    if (resource)
    {
        debugMessage("Loading configuration");
        storage.readResource(resource->getHeader(), &configuration, 0, sizeof(configuration));
    }
    else
    {
        debugMessage("No configuration found!");
        needsConfiguration = true;

        // Provide sensible default values.
        configuration = {
            false, // mod_attenuverters
            false, // cv_attenuverters
            0, // revision
        };
    }
}

void setUncalibratedParameterValue(uint8_t pid, int16_t value)
{
    int16_t previous = getParameterValue(pid);
    // IIR exponential filter with lambda 0.75: y[n] = 0.75*y[n-1] + 0.25*x[n]
    value = (float)((previous * 3 + value) >> 2);

    setParameterValue(pid, value);
}

void setMux(uint8_t index)
{
    muxA.set(index & 0b001);
    muxB.set(index & 0b010);
    muxC.set(index & 0b100);
}

void readMux(uint8_t index, uint16_t *mux_values)
{
    uint16_t muxA = 4095 - mux_values[MUX_A]; // RESONATORCV
    uint16_t muxB = 4095 - mux_values[MUX_B]; // Multiplexed params
    uint16_t muxC = 4095 - mux_values[MUX_C]; // MOD_LEVEL
    uint16_t muxD = 4095 - mux_values[MUX_D]; // MOD_SPEED
    uint16_t muxE = 4095 - mux_values[MUX_E]; // FILTERCV
    uint16_t muxF = 4095 - mux_values[MUX_F]; // INLEVELGREEN_LED // This will be DAC in rev 2 

    setUncalibratedParameterValue(RESONATORCV, muxA);
    setUncalibratedParameterValue(PARAMETER_BA + index, muxB);
    setUncalibratedParameterValue(MOD_LEVEL, muxC);
    setUncalibratedParameterValue(MOD_SPEED, muxD);
    setUncalibratedParameterValue(FILTERCV, muxE);
    setUncalibratedParameterValue(INLEVELGREEN_LED, muxF);
}

extern "C"
{
    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
    {
        extern ADC_HandleTypeDef ADC_PERIPH;
        extern ADC_HandleTypeDef MUX_PERIPH;
        extern uint16_t adc_values[NOF_ADC_VALUES];
        if (hadc == &MUX_PERIPH)
        {
            static uint8_t mux_index = 0;
            setMux(mux_index + 1);
            readMux(mux_index, mux_values);
            mux_index = (mux_index + 1) & 0b111;
        }
        else if (hadc == &ADC_PERIPH)
        {
            for (size_t i = 0; i < NOF_ADC_VALUES; i++)
            {
                uint16_t value = 4095 - adc_values[i];
                setUncalibratedParameterValue(i, value);
            }
        }
    }
}

void readGpio()
{
    if (randomButtonState != !randomButton.get()) // Inverted: pressed = false
    {
        randomButtonState = !randomButton.get();
        setButtonValue(RANDOM_BUTTON, randomButtonState);
        if (CONFIG_MODE_OPTIONS == configMode && randomButtonState)
        {
            configuration.cv_attenuverters = !configuration.cv_attenuverters;
        }
    }
    if (randomMapButtonState != !randomMapButton.get()) // Inverted: pressed = false
    {
        randomMapButtonState = !randomMapButton.get();
        setButtonValue(RANDOM_MAP, randomMapButtonState);
        if (CONFIG_MODE_OPTIONS == configMode && randomMapButtonState)
        {
            configuration.mod_attenuverters = !configuration.mod_attenuverters;
        }
    }
    if (shiftButtonState != !shiftButton.get()) // Inverted: pressed = false
    {
        shiftButtonState = !shiftButton.get();
        setButtonValue(SHIFT_BUTTON, shiftButtonState);
    }
    if (modCvButtonState != !modCvButton.get()) // Inverted: pressed = false
    {
        modCvButtonState = !modCvButton.get();
        setButtonValue(MOD_CV_BUTTON, modCvButtonState);
    }

    setAnalogValue(MOD_LED, getParameterValue(MOD));
    //setAnalogValue(INLEVELGREEN_LED, getParameterValue(INLEVELGREEN));

#ifdef DEBUG

    int16_t delayCv = getParameterValue(DELAYCV);            // Ok (-5 - 10v)
    int16_t osc2Cv = getParameterValue(OSC_DETUNE_CV);             // Ok (-5 - 10v)
    int16_t filterCv = getParameterValue(FILTERCV);        // Ok (-5 - 10v)
    int16_t startCv = getParameterValue(LOOPER_START_CV);          // Ok (-5 - 10v)
    int16_t lengthCv = getParameterValue(LOOPER_LENGTH_CV);        // Ok (-5 - 10v)
    int16_t resonatorCv = getParameterValue(RESONATORCV); // Ok (-5 - 10v)
    int16_t speedCv = getParameterValue(LOOPER_SPEED_CV);          // Ok (-5 - 10v)
    int16_t reverbCv = getParameterValue(REVERB_TONESIZE_CV);      // Ok (-5 - 10v)
    int16_t vOctCv = getParameterValue(OSC_VOCT_CV);               // Ok (0 - 10v)

    int16_t looperVol = getParameterValue(LOOPER_VOL);
    int16_t reverbVol = getParameterValue(REVERB_VOL);

    int16_t delayVol = getParameterValue(DELAY_VOL);
    int16_t resoVol = getParameterValue(RESONATOR_VOL);
    int16_t filterVol = getParameterValue(FILTER_VOL);
    int16_t inVol = getParameterValue(IN_VOL);
    int16_t sswtVol = getParameterValue(SSWT_VOL);
    int16_t sineVol = getParameterValue(SINE_VOL);

    int16_t speed = getParameterValue(LOOPER_SPEED);
    int16_t resoD = getParameterValue(FILTER_RESODRIVE);
    int16_t detune = getParameterValue(OSC_DETUNE);
    int16_t length = getParameterValue(LOOPER_LENGTH);
    int16_t pitch = getParameterValue(OSC_PITCH);
    int16_t start = getParameterValue(LOOPER_START);
    int16_t resoHarmony = getParameterValue(RESONATOR_HARMONY);
    int16_t resoDecay = getParameterValue(RESONATOR_DECAY);
    int16_t toneSize = getParameterValue(REVERB_TONESIZE);
    int16_t decay = getParameterValue(REVERB_DECAY);
    int16_t cutoff = getParameterValue(FILTER_CUTOFF);

    int16_t delayF = getParameterValue(DELAY_FEEDBACK);
    int16_t delayA = getParameterValue(DELAY_TIME);
    int16_t randomMode = getParameterValue(RANDOM_MODE);

    int16_t modAmount = getParameterValue(MOD_LEVEL);
    int16_t modFreq = getParameterValue(MOD_FREQ);

#endif
}

void onChangePin(uint16_t pin)
{
    switch (pin)
    {
    case SYNCIN_Pin:
    {
        bool state = HAL_GPIO_ReadPin(SYNCIN_GPIO_Port, SYNCIN_Pin) == GPIO_PIN_RESET; // Inverted
        setButtonValue(SYNC_GATE, state);
        break;
    }
    case RANDOMGATEIN_Pin:
    {
        bool state = HAL_GPIO_ReadPin(RANDOMGATEIN_GPIO_Port, RANDOMGATEIN_Pin) == GPIO_PIN_RESET; // Inverted
        setButtonValue(RANDOM_GATE, state);
        break;
    }
    }
}

void setAnalogValue(uint8_t ch, int16_t value)
{
    extern DAC_HandleTypeDef DAC_HANDLE;
    switch (ch)
    {
    case INLEVELLEDGREEN:
        HAL_DAC_SetValue(&DAC_HANDLE, DAC_CHANNEL_1, DAC_ALIGN_12B_R, __USAT(value, 12));
        break;
    case MOD:
        HAL_DAC_SetValue(&DAC_HANDLE, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095 - __USAT(value, 12)); // Inverted
        break;
    }
}

void setGateValue(uint8_t ch, int16_t value)
{
    switch (ch)
    {
    case RANDOM_BUTTON:
        setLed(RANDOM_LED, value);
        break;
    case RANDOM_MAP:
        setLed(RANDOM_MAP_LED, value);
        break;
    case SYNC_GATE:
        setLed(SYNC_LED, value);
        break;
    case INLEVELRED:
        setLed(INLEVELRED_LED, value);
        break;
    case SHIFT_BUTTON:
        setLed(SHIFT_LED, value);
        break;
    case MOD_CV_GREEN:
        setLed(MOD_CV_GREEN_LED, value);
        break;
    case MOD_CV_RED:
        setLed(MOD_CV_RED_LED, value);
        break;
    case IN_DETEC:
        HAL_GPIO_WritePin(IN_DETEC_GPIO_Port, IN_DETEC_Pin, value ? GPIO_PIN_RESET :  GPIO_PIN_SET);
        break;
    }
}

void setLed(uint8_t led, uint32_t rgb)
{
    switch (led)
    {
    case RANDOM_LED:
        HAL_GPIO_WritePin(RANDOM_BUTTONLED_GPIO_Port, RANDOM_BUTTONLED_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case RANDOM_MAP_LED:
        HAL_GPIO_WritePin(RND_MAP_BUTTONLED_GPIO_Port, RND_MAP_BUTTONLED_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case SYNC_LED:
        HAL_GPIO_WritePin(SYNCLED_GPIO_Port, SYNCLED_Pin, rgb == NO_COLOUR ? GPIO_PIN_SET : GPIO_PIN_RESET); // Inverted
        break;
    case INLEVELRED_LED:
        HAL_GPIO_WritePin(INLEVELREDLED_GPIO_Port, INLEVELREDLED_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case SHIFT_LED:
        HAL_GPIO_WritePin(SHIFT_BUTTONLED_GPIO_Port, SHIFT_BUTTONLED_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case MOD_CV_GREEN_LED:
        HAL_GPIO_WritePin(MOD_AMT_BUTTON_LED_2_GPIO_Port, MOD_AMT_BUTTON_LED_2_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case MOD_CV_RED_LED:
        HAL_GPIO_WritePin(MOD_AMT_BUTTON_LED_GPIO_Port, MOD_AMT_BUTTON_LED_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    }
}

void ledsOn()
{
    setLed(RANDOM_LED, 1);
    setLed(RANDOM_MAP_LED, 1);
    setLed(SYNC_LED, 1);
    setLed(INLEVELRED_LED, 1);
    setLed(SHIFT_LED, 1);
    setLed(MOD_CV_RED_LED, 1);
    setAnalogValue(MOD_LED, 4095);
}

void ledsOff()
{
    setLed(RANDOM_LED, NO_COLOUR);
    setLed(RANDOM_MAP_LED, NO_COLOUR);
    setLed(SYNC_LED, NO_COLOUR);
    setLed(INLEVELRED_LED, NO_COLOUR);
    setLed(SHIFT_LED, NO_COLOUR);
    setLed(MOD_CV_GREEN_LED, NO_COLOUR);
    setLed(MOD_CV_RED_LED, NO_COLOUR);
    setAnalogValue(MOD_LED, NO_COLOUR);
}

void updateParameters(int16_t *parameter_values, size_t parameter_len, uint16_t *adc_values, size_t adc_len)
{
    //uint8_t value = (randomAmountSwitch2.get() << 1) | randomAmountSwitch1.get();
    //parameter_values[RANDOM_AMOUNT] = 2047 * (value - 1); // Mid = 0, Low = 2047, High = 4094
}

void onSetup()
{
    extern ADC_HandleTypeDef MUX_PERIPH;
    if (HAL_ADC_Start_DMA(&MUX_PERIPH, (uint32_t *)mux_values, NOF_MUX_VALUES) != HAL_OK)
    {
        error(CONFIG_ERROR, "ADC1 Start failed");
    }

    onChangePin(SYNCIN_Pin);
    onChangePin(RANDOMGATEIN_Pin);

    setGateValue(IN_DETEC, 0);

    loadConfiguration();
}

void onLoop(void)
{
    static uint32_t counter = PATCH_RESET_COUNTER;

    bool shiftButtonPressed = HAL_GPIO_ReadPin(SHIFT_BUTTON_GPIO_Port, SHIFT_BUTTON_Pin) == GPIO_PIN_RESET;
    bool modCvButtonPressed = HAL_GPIO_ReadPin(MOD_AMT_BUTTON_GPIO_Port, MOD_AMT_BUTTON_Pin) == GPIO_PIN_RESET;
    bool rndButtonPressed = HAL_GPIO_ReadPin(RANDOM_BUTTON_GPIO_Port, RANDOM_BUTTON_Pin) == GPIO_PIN_RESET;
    bool rndMapButtonPressed = HAL_GPIO_ReadPin(RND_MAP_BUTTON_GPIO_Port, RND_MAP_BUTTON_Pin) == GPIO_PIN_RESET;
    static bool buttonPressed = false;

    if (RUN_MODE != owl.getOperationMode())
    {
        if (shiftButtonPressed && modCvButtonPressed)
        {
            configMode = CONFIG_MODE_OPTIONS;
            buttonPressed = true;
        }
    }

    switch (owl.getOperationMode())
    {
    case LOAD_MODE:
        // Called on patch load.
        if (CONFIG_MODE_NONE == configMode)
        {
            ledsOff();
            if (getErrorStatus() != NO_ERROR)
            {
                owl.setOperationMode(ERROR_MODE);
            }
        }
        else
        {
            configMode = CONFIG_MODE_NONE;

            // Need to turn the MOD/CV button off before restarting the patch...
            modCvButtonState = 0;
            setButtonValue(MOD_CV_BUTTON, modCvButtonState);

            // Restart the patch.
            program.resetProgram(false);
            owl.setOperationMode(RUN_MODE);
        }
        break;
    case RUN_MODE:
        if (CONFIG_MODE_NONE == configMode)
        {
            readGpio();
            if (getErrorStatus() != NO_ERROR)
            {
                owl.setOperationMode(ERROR_MODE);
            }
        }
        else
        {
            // Stop the patch and enter configuration mode.
            program.exitProgram(true);
            owl.setOperationMode(CONFIGURE_MODE);
        }
        break;

    case CONFIGURE_MODE:
        if (CONFIG_MODE_NONE != configMode)
        {
            if (buttonPressed && !rndMapButtonPressed && !rndButtonPressed && !shiftButtonPressed && !modCvButtonPressed)
            {
                buttonPressed = false;
            }
            if (!buttonPressed)
            {
                if (needsConfiguration)
                {
                    // Reset configuration before calibration.
                    configuration = {
                        false,  // mod_attenuverters
                        false,  // cv_attenuverters
                        0,      // revision
                    };

                    needsConfiguration = false;
                }

                if (CONFIG_MODE_OPTIONS)
                {
                    readGpio();
                    setLed(RANDOM_LED, configuration.mod_attenuverters);
                    setLed(RANDOM_MAP_LED, configuration.cv_attenuverters);
                    setLed(MOD_CV_GREEN_LED, 1);

                    if (modCvButtonPressed)
                    {
                        ledsOff();
                        // Save and exit calibration.
                        saveConfiguration();
                        owl.setOperationMode(LOAD_MODE);
                    }
                }
            }
        }
        else
        {
            // Config button has not been pressed during startup, go to load mode.
            owl.setOperationMode(LOAD_MODE);
        }
        break;

    case ERROR_MODE:
        ledsOn();
        if (shiftButtonPressed)
        {
            if (--counter == 0)
            {
                // Reset device after error.
                ledsOff();
                setErrorStatus(NO_ERROR);
                owl.setOperationMode(STARTUP_MODE);
                program.resetProgram(false);
            }
        }
        else
        {
            const char *message = getErrorMessage();
            counter = PATCH_RESET_COUNTER;
        }
        break;
    }
}

#define MAX_PATCH_SETTINGS 16 // Max number of available MIDI channels
#define PATCH_SETTINGS_NAME "iroi"

int16_t data[MAX_PATCH_SETTINGS] = {};
uint8_t fileIndex = 0;

bool onMidiSend(uint8_t port, uint8_t status, uint8_t d1, uint8_t d2)
{
    MidiMessage msg(port, status, d1, d2);

    if (msg.isPitchBend())
    {
        int ch = msg.getChannel();
        if (ch < MAX_PATCH_SETTINGS)
        {
            data[ch] = msg.getPitchBend();
        }
    }
    else if (msg.data[1] == START)
    {
        // clear settings
        memset(data, 0, sizeof(data));

        return false; // suppress this message
    }
    else if (msg.isChannelPressure())
    {
        fileIndex = msg.getChannelPressure();

        return false; // suppress this message
    }
    else if (msg.data[1] == STOP)
    {
        const char *filename;
        switch (fileIndex)
        {
        case 0:
            filename = PATCH_SETTINGS_NAME ".prm";
            break;
        case 1:
            filename = PATCH_SETTINGS_NAME ".alt";
            break;
        case 2:
            filename = PATCH_SETTINGS_NAME ".mod";
            break;
        case 3:
            filename = PATCH_SETTINGS_NAME ".cv";
            break;
        case 4:
            filename = PATCH_SETTINGS_NAME ".rnd";
            break;
        }

        uint32_t headerSize = sizeof(ResourceHeader);
        uint32_t dataSize = sizeof(data);
        uint8_t buffer[headerSize + dataSize];
        memset(buffer, 0, headerSize);
        memcpy(buffer + headerSize, data, dataSize);
        taskENTER_CRITICAL(); // This causes the audio to glitch
        storage.writeResource(filename, buffer, dataSize, FLASH_DEFAULT_FLAGS);
        taskEXIT_CRITICAL();

        return false; // suppress this message
    }

    return true;
}

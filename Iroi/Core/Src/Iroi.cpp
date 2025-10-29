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

#define MAP_SELECTOR PARAMETER_AE
#define INLEVELGREEN PARAMETER_AF
#define MOD          PARAMETER_AG

// GPIO
#define RANDOM_BUTTON BUTTON_1
#define MAP_BUTTON    BUTTON_2
#define RANDOM_GATE   BUTTON_3
#define SYNC_GATE     BUTTON_4
#define INLEVELRED    BUTTON_5
#define SHIFT_BUTTON  BUTTON_6
#define IN_DETEC      BUTTON_7

// ADC3
#define FILTER_LEVEL  PARAMETER_A
#define RESO_LEVEL    PARAMETER_B
#define DELAY_LEVEL   PARAMETER_C
#define REVERB_LEVEL  PARAMETER_D
#define FILTER_CUTOFF PARAMETER_E
#define DELAYCV       PARAMETER_F
#define REVERBCV      PARAMETER_G

// ADC1
#define RESONATORCV     PARAMETER_H
#define MOD_LEVEL       PARAMETER_AA
#define MOD_SPEED       PARAMETER_AB
#define FILTERCV        PARAMETER_AC
#define INLEVELLEDGREEN PARAMETER_AD

// ADC1 muxed (pots)
#define FILTER_RESONANCE   PARAMETER_BF
#define RESONATOR_TUNE     PARAMETER_BG
#define RESONATOR_FEEDBACK PARAMETER_BB
#define ECHO_DENSITY       PARAMETER_BE
#define ECHO_REPEATS       PARAMETER_BD
#define AMBIENCE_SPACETIME PARAMETER_BA
#define AMBIENCE_DECAY     PARAMETER_BH

enum leds
{
    RANDOM_BUTTON_LED = 1,
    MAP_BUTTON_LED,
    SYNC_LED,
    INLEVELRED_LED,
    INLEVELGREEN_LED,
    MOD_LED,
    SHIFT_LED,
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
static bool mapButtonState = false;
static bool shiftButtonState = false;
static uint16_t mux_values[NOF_MUX_VALUES] DMA_RAM = {};

Pin randomButton(RANDOM_BUTTON_GPIO_Port, RANDOM_BUTTON_Pin);
Pin mapButton(RND_MAP_BUTTON_GPIO_Port, RND_MAP_BUTTON_Pin);
Pin shiftButton(SHIFT_BUTTON_GPIO_Port, SHIFT_BUTTON_Pin);
Pin mapSelectorSwitch1(RANDOMAMOUNTSW_GPIO_Port, RANDOMAMOUNTSW_Pin);
Pin mapSelectorSwitch2(RANDOMAMOUNTSW2_GPIO_Port, RANDOMAMOUNTSW2_Pin);

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
    uint16_t muxA = 4095 - mux_values[MUX_A]; // MOD_LEVEL // 14
    uint16_t muxB = 4095 - mux_values[MUX_B]; // MOD_SPEED // 15
    uint16_t muxC = 4095 - mux_values[MUX_C]; // RESONATORCV // 16
    uint16_t muxD = 4095 - mux_values[MUX_D]; // Multiplexed params // 17
    uint16_t muxE = 4095 - mux_values[MUX_E]; // FILTERCV // 5

    setUncalibratedParameterValue(MOD_LEVEL, muxA);
    setUncalibratedParameterValue(MOD_SPEED, muxB);
    setUncalibratedParameterValue(RESONATORCV, muxC);
    setUncalibratedParameterValue(PARAMETER_BA + index, muxD);
    setUncalibratedParameterValue(FILTERCV, muxE);
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
                int16_t value = 4095 - adc_values[i];
                if (i >= 0 && i <= 3) 
                {
                    // Faders have a 5V range.
                    value -= 1300; // ~ 4095 / 15 * 5
                    if (value < 0) value = 0;
                }
                setUncalibratedParameterValue(i, value);
            }

            //  0 > Filter fader
            //  1 > Resonator fader
            //  2 > Echo fader
            //  3 > Ambience fader
            //  4 > Filter cutoff (0 - 4037)
            //  5 > Echo cv (-5v - 10v)
            //  6 > Ambience cv
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
    if (mapButtonState != !mapButton.get()) // Inverted: pressed = false
    {
        mapButtonState = !mapButton.get();
        setButtonValue(MAP_BUTTON, mapButtonState);
        if (CONFIG_MODE_OPTIONS == configMode && mapButtonState)
        {
            configuration.mod_attenuverters = !configuration.mod_attenuverters;
        }
    }
    if (shiftButtonState != !shiftButton.get()) // Inverted: pressed = false
    {
        shiftButtonState = !shiftButton.get();
        setButtonValue(SHIFT_BUTTON, shiftButtonState);
    }

    setAnalogValue(MOD_LED, getParameterValue(MOD));
    setAnalogValue(INLEVELGREEN_LED, getParameterValue(INLEVELGREEN));
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
        setLed(RANDOM_BUTTON_LED, value);
        break;
    case MAP_BUTTON:
        setLed(MAP_BUTTON_LED, value);
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
    case IN_DETEC:
        HAL_GPIO_WritePin(IN_DETEC_GPIO_Port, IN_DETEC_Pin, value ? GPIO_PIN_RESET :  GPIO_PIN_SET);
        break;
    }
}

void setLed(uint8_t led, uint32_t rgb)
{
    switch (led)
    {
    case RANDOM_BUTTON_LED:
        HAL_GPIO_WritePin(RANDOM_BUTTONLED_GPIO_Port, RANDOM_BUTTONLED_Pin, rgb == NO_COLOUR ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case MAP_BUTTON_LED:
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
    }
}

void ledsOn()
{
    setLed(RANDOM_BUTTON_LED, 1);
    setLed(MAP_BUTTON_LED, 1);
    setLed(SYNC_LED, 1);
    setLed(INLEVELRED_LED, 1);
    setLed(SHIFT_LED, 1);
    setAnalogValue(MOD_LED, 4095);
}

void ledsOff()
{
    setLed(RANDOM_BUTTON_LED, NO_COLOUR);
    setLed(MAP_BUTTON_LED, NO_COLOUR);
    setLed(SYNC_LED, NO_COLOUR);
    setLed(INLEVELRED_LED, NO_COLOUR);
    setLed(SHIFT_LED, NO_COLOUR);
    setAnalogValue(MOD_LED, NO_COLOUR);
}

void updateParameters(int16_t *parameter_values, size_t parameter_len, uint16_t *adc_values, size_t adc_len)
{
    uint8_t value = (mapSelectorSwitch2.get() << 1) | mapSelectorSwitch1.get();
    parameter_values[MAP_SELECTOR] = 2047 * (value - 1); // Mid = 0, Low = 2047, High = 4094
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
    bool rndButtonPressed = HAL_GPIO_ReadPin(RANDOM_BUTTON_GPIO_Port, RANDOM_BUTTON_Pin) == GPIO_PIN_RESET;
    bool mapButtonPressed = HAL_GPIO_ReadPin(RND_MAP_BUTTON_GPIO_Port, RND_MAP_BUTTON_Pin) == GPIO_PIN_RESET;
    static bool buttonPressed = false;

    if (RUN_MODE != owl.getOperationMode())
    {
        if (shiftButtonPressed)
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
            if (buttonPressed && !mapButtonPressed && !rndButtonPressed && !shiftButtonPressed)
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
                    setLed(RANDOM_BUTTON_LED, configuration.mod_attenuverters);
                    setLed(MAP_BUTTON_LED, configuration.cv_attenuverters);

                    if (mapButtonPressed)
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

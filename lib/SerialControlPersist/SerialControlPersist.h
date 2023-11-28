#ifndef SERIAL_CONTROL_PERSIST_LIB
#define SERIAL_CONTROL_PERSIST_LIB

#include <Arduino.h>
#include <map>

/**
 * @class SerialControlPersist
 * @brief The SerialControlPersist class provides functionality for handling persistent serial communication.
 *
 * This class enables the control of variables through serial communication. It allows setting and updating
 * integer and float variables dynamically via messages received through a specified serial Stream.
 */
class SerialControlPersist
{
private:
    std::map<String, int *> registredINTs;                            /**< Map of registered integer pointers */
    std::map<String, float *> registredFLOATs;                        /**< Map of registered float pointers */
    std::map<String, void (*)()> registeredVOIDs_withNoArg;           /**< Map of registered functions with no arguments. */
    std::map<String, void (*)(String)> registeredVOIDs_WithStringArg; /**< Map of registered functions with a string argument. */
    String incomingBuffer = "";                                       /**< Holds all incoming bytes until a <;> is received. */
    String incomingMessage = "";                                      /**< Takes the buffer content whenever a <;> is received. */
    String separator = "-";                                           /**< The separator between key and value. */

    Stream *BluetoothSerial = nullptr; /**< Default serial Stream for communication. */
    bool serial_initialized = false;   /**< Flag indicating whether the serial Stream has been initialized. */

public:
    String lastKey = "";   /**< Last key received from a valid message. */
    String lastValue = ""; /**< Last value received from a valid message. */

    /**
     * @brief Default constructor for SerialControlPersist.
     */
    SerialControlPersist();

    /**
     * @brief Constructor for SerialControlPersist with a specified serial Stream and separator.
     * @param s The serial Stream to use for communication.
     * @param separator The separator string between key and value in incoming messages.
     */
    SerialControlPersist(Stream *s, String separator = "-");

    /**
     * @brief Destructor for SerialControlPersist.
     */
    ~SerialControlPersist();

    /**
     * @brief Sets the serial Stream for communication.
     * @param s A pointer to the Serial stream.
     */
    void setSerial(Stream *s);

    /**
     * @brief Sets the separator string between key and value in incoming messages.
     * @param s The separator string to set.
     */
    void setSeparator(String s);

    /**
     * @brief Registers an integer variable for remote control.
     * @param key The unique identifier for the variable.
     * @param ptr Pointer to the integer variable to be controlled.
     */
    void registerINT(String key, int *ptr);

    /**
     * @brief Registers a float variable for remote control.
     * @param key The unique identifier for the variable.
     * @param ptr Pointer to the float variable to be controlled.
     */
    void registerFLOAT(String key, float *ptr);

    /**
     * @brief Registers a void function with no arguments for remote control.
     * @details This method allows registering a void function that does not take any arguments,
     *          enabling remote execution of specific actions without parameters.
     * @param key The unique identifier for the void function.
     * @param ptr Pointer to the void function with no arguments.
     * @note Ensure that the provided function matches the void (*)() signature.
     */
    void registerVoidNoArg(String key, void (*ptr)());

    /**
     * @brief Registers a void function with a String argument for remote control.
     * @details This method allows registering a void function that takes a String argument,
     *          enabling remote execution of specific actions with a parameter.
     * @param key The unique identifier for the void function.
     * @param ptr Pointer to the void function that takes a String argument.
     * @note Ensure that the provided function matches the void (*)(String) signature.
     */
    void registerVoidWithStringArg(String key, void (*ptr)(String));

    bool isAvailable(String key);

    /**
     * @brief Reads incoming serial data and processes it.
     * @return True if a complete message is received, false otherwise.
     */
    bool readSerial();

    /**
     * @brief Updates registered variables with the latest values received through serial communication.
     * @return True if variables are updated, false otherwise.
     */
    bool update();
};

#endif

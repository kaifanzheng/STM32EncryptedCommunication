# STM32EncryptedCommunication
The design problem for this project was to implement a secure two-way communication system, for personal or confidential data to be sent. Many encryption methods already exist, some being more secure than others. For this project, we chose the tried and true RSA encryption method that relies on prime numbers for its encryption mechanism. 

## Project structure
<p align=center>
    <img src="https://github.com/Noshin03/STM32EncryptedCommunication/blob/main/ImageResource/overview.png">
</p>

### RSA
The RSA Cryptosystem encrypts messages for communication using a set of public and private keys. Only the receiver knows the private key, but the public key is visible to everyone, using which anyone can encrypt a message to send to the receiver, but since no one but the receiver knows the private key, only the receiver can decrypt the message, thus making it secure from intruders. The system works with the mathematical concept that modular arithmetic is difficult to inverse. To form the private key, we can use two prime numbers. The larger the primes, the more secure the system is since it is more difficult to reverse the mod of a larger number. The algorithm is summarized in the following diagram.
<p align=center>
    <img src="https://github.com/Noshin03/STM32EncryptedCommunication/blob/main/ImageResource/RSA.png">
</p>

### UART Communication Driver 
To communicate between two or more boards, we are using serial communication with wires through UART ports on the STM-32 boards. We decided on using UART after our extensive research and attempts on using Wi-Fi for communication proved to be unsuccessful. We use UART protocols and driver functions from HAL library to program the transmission of data by utilizing the RX and TX pins which are for receiving and transmitting information respectively. We configure the pins in MX using the same baud rate of 9600bit/s for all boards in the network. During transmission and reception UART converts raw data into data packets, which during integration of the Cryptosystem we discovered to be 8-bits. We were unable to transmit encrypted messages since the encrypted value can be much larger than the original message, giving larger than 8-bit values for our test messages. To solve this, we converted the numbers to a string which is essentially a char array, to be transmitted. We separated each number with a ‘.’as a separator, which upon reception by the receiving board can be split into the different numbers as strings and converted to integers to be decrypted.

### Temperature and humidity sensor functionality
The temperature and humidity sensor (HTS221) also remained unchanged from the initial demo and report. The BL4S5I board support package is used to access the sensor and read the temperature and humidity data. In order to access this data as the user, one board sends ‘B’ for temperature or ‘C’ for humidity to another board that’s been paired. In return, the other board sends back the temperature and humidity values. This design choice was made so that the temperature data is logically available to the user. For instance, if the two boards are reasonably far away from each other, a user of the system can see temperature data from another location without physically moving. Otherwise, there was not much design involved with this part of the project. The sensor works as expected and outputs the temperature and the output is shown correctly when requested by a user. 

### Speaker Output (ringtone) 
The ringtone remained the same throughout the project, simply consisting of 4 tones and their harmonics. The entire ringtone is generated when the board starts up and is stored in memory. Since the array is of length 22932 with 8 bits per sample, the array can be safely stored in memory without running into allocation issues for other components. The ringtone being stored in memory also allows the DAC to output the waveform through DMA, driven by a 44.1kHz timer, without talking up many CPU resources. This was done because we knew we had the capacity for memory to store the ringtone, and also wanted the system to be as responsive as possible since user input/output is a big part of the project. However, an alternate, even less impactful method of saving the ringtone is to store it in code as a predefined array. The reason this is not done in our project is because we wanted to retain a dynamic generation process, so that the ringtone would be easy to change later. Also, doing this would negatively impact the readability of our code. In terms of results, the connected piezoelectric buzzer works as expected and plays back the four notes in the correct order at 44.1kHz. Performance impact apart from generating the ringtone at the start is minimal. 

### 4x4 Membrane Keyboard Functionality 
The membrane keyboard works based on polling. This was not so much a design decision as it was a necessity. The keyboard works by detecting a change in one of the row outputs, and the column is checked by asserting a signal and seeing if the response is active or not. With this, the keyboard is not very well suited for interrupts, as determining the column is inherently a polling-like action. Each key is mapped as follows: 
<p align=center>
    <img src="https://github.com/Noshin03/STM32EncryptedCommunication/blob/main/ImageResource/keyboard.png">
</p>

### OLED Screen Output 
The OLED screen is driven by a library created by N. Mohideen​ [1]​. This library allows the program to print characters in varying sizes onto the screen. With this driver, we were able to create a menu on the screen where the user can choose the functionality. For example, the user can view a history of messages received, send a message or pair to another device. The complete flow of the UI is demonstrated below:
<p align=center>
    <img src="https://github.com/Noshin03/STM32EncryptedCommunication/blob/main/ImageResource/state_machine.png">
</p>

### Improved RSA Performance Through Assembly 
RSA involves a lot of computation in order to encrypt and decrypt data. This makes it a good candidate for improvement through arm assembly instructions instead of the previous C implementation.We translated three functions from C to assembly: emod, isCoprime, and isPrime.
<p align=center>
    <img src="https://github.com/Noshin03/STM32EncryptedCommunication/blob/main/ImageResource/runtime.png">
</p>

## Conclusion
Our system could be used for 2-way communication between two physically connected locations. Additionally, as the system is able to transmit temperature data, it could be used for a weather prediction network. Our boards use a wired UART connection, but it might be more applicable to use something like the LoRa module for wireless communication. This would enable better range for the boards to communicate, up to 15km as specified by the LoRa documentation ​[2]​. In contrast, with our current wired communication, this would be very difficult to achieve and maintain. 

More details please refer to the report folder

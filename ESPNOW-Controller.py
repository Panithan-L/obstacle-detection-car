from machine import Pin, DAC, ADC, PWM, Timer
from time import sleep, ticks_ms
import network
import espnow

'''Declaring the Pins for the Joysticks and Joystick Buttons'''
# Set pins A2 (GPIO34) and A3 (GPIO26) for the right and left joystick 
right_control = Pin(34, mode=Pin.IN)
left_control = Pin(39, mode=Pin.IN)
# Set pins GPIO12 and GPIO15 for the right and left joystick buttons
right_button = Pin(12, mode=Pin.IN, pull=Pin.PULL_UP)
left_button = Pin(15, mode=Pin.IN, pull=Pin.PULL_UP)

'''Initializing the Joysticks'''
# Assign ADC converter objects to each pin
adc_right = ADC(right_control)
adc_left = ADC(left_control)
# ADC configurations
adc_right.atten(ADC.ATTN_11DB)  	# change range of converter to be V_ref = 3.2
adc_left.atten(ADC.ATTN_11DB)

'''Initializing the Joystick Buttons'''
# Set varibles for the button and button handler
counter = 0
state = 0
last_time = ticks_ms()
t = 0 			# dummy value
delta_t = 0 	# dummy value


'''Initialize Pins for LEDs'''
leftLEDPin = Pin(14, Pin.OUT)
leftLED = PWM(leftLEDPin, freq=5, duty=1023)

centerLEDPin = Pin(32, Pin.OUT)
centerLED = PWM(centerLEDPin, freq=5, duty=1023)

rightLEDPin = Pin(27, Pin.OUT)
rightLED = PWM(rightLEDPin, freq=5, duty=1023)

# Distance adjustments
maxDist = 30    # 25% LED brightness
medDist = 15    # 50% LED brightness
closeDist = 3  # 100% LED brightness

LED_Left = 40
LED_Center = 40
LED_Right = 40

# ISR handler function when left button is pressed
def bhandler_left(pin):
    global counter, state
    global last_time, t, delta_t
    
    # Debounce code (from lecture)
    state = 0
    t = ticks_ms()
    delta_t = t - last_time
    # delta_t is in ms
    if (delta_t > 20) and (left_button() == 1):
        state = left_button()
        last_time = t
        counter += 1

# ISR handler function when right button is pressed
def bhandler_right(pin):
    global counter, state
    global last_time, t, delta_t
    
    # Debounce code (from lecture)
    state = 0
    t = ticks_ms()
    delta_t = t - last_time
    # delta_t is in ms
    if (delta_t > 20) and (right_button() == 1):
        state = left_button()
        last_time = t
        counter += 1


def recv_cb(e):
    while True:  # Read out all messages waiting in the buffer
        mac, msg = e.irecv(0)  # Don't wait if no messages left
        if mac is None:
            return
        print(mac, msg)

# Initialize pin with ISR
left_button.irq(handler = bhandler_left, trigger = Pin.IRQ_RISING)
right_button.irq(handler = bhandler_right, trigger = Pin.IRQ_RISING)


'''ESPNOW Variables to be sent'''
right_wheel = 0
left_wheel = 0

'''ESPNOW Section - Sending Data'''
# ESPNOW section
# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.STA_IF)  # Or network.AP_IF
sta.active(True)
sta.disconnect()      				# For ESP8266

e = espnow.ESPNow()
e.active(True)
# Different MAC Addresses
# Rhonin ESP MAC Address: 	e8-9f-6d-32-db-b0
# Panithan ESP Mac Address: e8-9f-6d-32-da-70

rhonin_peer = 	b'\xe8\x9f\x6d\x32\xdb\xb0'
panithan_peer = b'\xe8\x9f\x6d\x32\xda\x70'
e.add_peer(rhonin_peer)      			# Must add_peer() beforee send()

e.send(rhonin_peer, "Starting...")

'''While Loop - runs continuously until either joystick button is pressed'''
# While loop, run continuously
while (counter < 1):
    
    # Storing the Joystick Values
    adc_val_right = adc_right.read_u16()
    adc_val_left = adc_left.read_u16()
    
    rightsw_val = right_button()
    leftsw_val = left_button()
        
    send_message = f",{adc_val_right},{adc_val_left}, end"
    print(send_message)
    
    # Message sent
    e.send(rhonin_peer, send_message, True)
    
    '''Receiving Message from Ultrasonic Sensor Readings'''
    while True:  # Read out all messages waiting in the buffer
        mac, msg = e.recv(0)  # Don't wait if no messages left
        if mac is None:
            break
        else:
            message = str(msg)
            values = message.split(',')
            LED_Left = int(values[1])
            LED_Center = int(values[2])
            LED_Right = int(values[3])
            
            ledUpdate = [LED_Left, LED_Center, LED_Right]
            print(ledUpdate)
            
                        
    '''Adjusting LED duty depending on distance'''
    #Left LED
    if ((LED_Left <= maxDist) and (LED_Left > medDist)):
        leftLED.duty(256)
    elif ((LED_Left <= medDist) and (LED_Left > closeDist)):
        leftLED.duty(512)
    elif (LED_Left <= closeDist):
        leftLED.duty(0)
    
    #Center LED
    if ((LED_Center <= maxDist) and (LED_Center > medDist)):
        centerLED.duty(256)
    elif ((LED_Center <= medDist) and (LED_Center > closeDist)):
        centerLED.duty(512)
    elif (LED_Center <= closeDist):
        centerLED.duty(0)
        
    #Right LED
    if ((LED_Right <= maxDist) and (LED_Right > medDist)):
        rightLED.duty(256)
    elif ((LED_Right <= medDist) and (LED_Right > closeDist)):
        rightLED.duty(512)
    elif (LED_Right <= closeDist):
        rightLED.duty(0)
    
    sleep(0.05)

'''Send Exit Message'''
end_message = f",30000,30000, end"


e.send(rhonin_peer, end_message, True)


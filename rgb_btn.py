import time
from xml.etree.ElementTree import PI
import RPi.GPIO as GPIO
import neopixel
import board

#User Params

BUTTON_PIN = 17         #declare the GPIO 23 pin for the BUTTON input
num_pixels = 6          #declare the number of NeoPixels
pixel_pin = board.D18   #declare the NeoPixel out put pin
BUTTON_PRESS_TIME = 5   #Button press state time
BUTTON_PRESS_MI_UPLOAD = 2 # This is for mission upload | starting the sequence!
BLINKING_TIME_SEQUENCES = 5

#Buzzer Pin
buzzer_pin = 19

#Warning: Do not change anything from here

GPIO.setmode(GPIO.BCM)  # for GPIO numbering, choose BCM
GPIO.setwarnings(False) # to disable warnings.
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(buzzer_pin, GPIO.OUT)

global buzz
buzz = GPIO.PWM(buzzer_pin, 1000)
buzz.start(0)

ORDER = neopixel.GRB
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.5, pixel_order=ORDER
)

def neo_pixel_color(r, g, b):
    pixels.fill((r, g, b))
    

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (b, g, r) if ORDER in (neopixel.RGB, neopixel.GRB) else (b, g, r, 0)

def rainbow_cycle(wait):

    for j in range(255):

        for i in range(num_pixels):

            pixel_index = (i * 255 // num_pixels) + j
            pixels[i] = wheel(pixel_index & 255)

        pixels.show()
        time.sleep(wait)


def parking_beep(wait_period):

    beep_sequence = [
        (1000, 0.5),  # Beep for 200ms at 1000Hz
                      
        ]

    count = 0
    # Loop through the beep sequence and play each tone
    while count < wait_period:

        for frequency, duration in beep_sequence:
            # Set the duty cycle for the given frequency
            buzz.ChangeFrequency(frequency)
            buzz.start(100)  # Start the PWM with a duty cycle of 50%
            
            # Wait for the given duration
            time.sleep(duration)
            
            # Stop the PWM output
            buzz.stop()
            
            # Pause before the next beep
            time.sleep(0.5)

        count += 1

def btn_hold_press(hold_period):

    parking_beep(wait_period=5)  #Indicates the operator, about time to press and hold the button for mission upload!

    GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)   #Waits for the ext interrupt aka Button Press!
    pixels.fill((255, 255, 255))
    print ("Button press detected")
    start = time.time()
    time.sleep(0.2)    # Button De-bounce!
    length = 0

    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.01)
        buzz.start(100)
        length = time.time() - start
        print(length)

        if length > hold_period:
            if hold_period == BUTTON_PRESS_MI_UPLOAD:
                pixels.fill((0, 255, 0))
                #buzzer 
                #buzz.ChangeDutyCycle(100)
                break
            else:
                pixels.fill((255, 0, 0))
                
                break

    return length

def btn_mission_upload():

    pixels.fill((0, 0, 0))
    length = 0
    while True:

        length = btn_hold_press(hold_period = BUTTON_PRESS_MI_UPLOAD)

        if length > BUTTON_PRESS_MI_UPLOAD:

            buzz.stop(0)
            print("First!!") 
            break
        
        else:
            pixels.fill((0, 0, 0))
            buzz.stop(0)

    print("Here!")

def btn_main():

    length = 0
    while True:

        length = btn_hold_press(hold_period = BUTTON_PRESS_TIME)


        if length > BUTTON_PRESS_TIME: 

            buzz.stop(0)

        #if button pressed time is greater than BUTTON_PRESS_TIME:value then next sequence will begin
            print ("Arming sequences started")

            for i in range(BLINKING_TIME_SEQUENCES):    
                #  this FOR loop is for, blinking LED interval of 500 miliseconds 
                    pixels.fill((255, 0, 0))
                    time.sleep(0.5) 

                    pixels.fill((0, 0, 255))
                    time.sleep(0.5)  

            pixels.fill((255, 0, 0))
            time.sleep(5)

            rainbow_cycle(0.01)

            parking_beep(wait_period=20)
            
            

            break 
            #the break is for, this code will not run again untill reboot
            GPIO.cleanup()

        else:
            pixels.fill((0, 0, 0))
            buzz.stop(0)


# btn_mission_upload()

# print("Mission Uploading....")
# print("Mission Uploading....")
# print("Mission Uploading....")
# print("Mission Uploading....")
# print("Mission Uploading....")
# print("Mission Uploading....")
# print("Mission Uploading....")

# btn_main()

# print("Mission Uploading....")

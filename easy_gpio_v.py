import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
pullup = GPIO.PUD_UP
pulldown = GPIO.PUD_DOWN
in_pins = []
out_pins = []
default_delay = 0.001

def outpin(pin):
    GPIO.setup(pin, GPIO.OUT)
    out_pins.append(pin)

def inpin(pin, pull_up_down=pulldown):
    GPIO.setup(pin, GPIO.IN, pull_up_down)
    GPIO.add_event_detect(pin, GPIO.BOTH, callback=print_state)
    in_pins.append(pin)

def outpins(pins):
    for pin in pins:
        outpin(pin)

def inpins(pins):
    for pin in pins:
        inpin(pin)

def up(pin):
    GPIO.output(pin, True)
    print_state(pin)

def down(pin):
    GPIO.output(pin, False)
    print_state(pin)

def ups(pins):
    for pin in pins:
        up(pin)

def downs(pins):
    for pin in pins:
        down(pin) 

def get(pin):
    return GPIO.input(pin)

def gets(pins):
    return [get(pin) for pin in pins]

def read(pin):
    return 'HIGH' if GPIO.input(pin) else 'LOW'

def reads(pins):
    return [read(pin) for pin in pins]

def print_state(pin):
    pin_type = GPIO.gpio_function(pin)
    if pin_type == GPIO.OUT:
        pin_type = 'OUT'
    elif pin_type == GPIO.IN:
        pin_type = 'IN'
    else:
        pin_type = str(pin_type)
    print(''.join(['Pin ', str(pin), ' (', pin_type, '): ',
                   read(pin)]))

def states():
    print('OUTPUTS:')
    for pin in out_pins:
        print_state(pin)
    print('INPUTS:')
    for pin in in_pins:
        print_state(pin)

def toggle(pin, num=1, delay=default_delay):
    GPIO.output(pin, not GPIO.input(pin))
    print_state(pin)

def step(pin, num=1, delay=default_delay):
    down(pin)
    for i in range(num):
        sleep(delay)
        up(pin)
        sleep(delay)
        down(pin)


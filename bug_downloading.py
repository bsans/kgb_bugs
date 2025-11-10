# Python3
# Button triggers random choice of audio to play


from gpiozero import Button, OutputDevice
import pygame.mixer
from pygame.mixer import Sound
from signal import pause
import random

pygame.mixer.pre_init(44100, -16, 2, 8192) # Adjust buffer size (last argument)
pygame.mixer.init()
button = Button(2, bounce_time=0.2)

def stop():
    pygame.mixer.music.stop()
    pygame.mixer.stop()

button_sounds = [
    Sound("/home/mcboatface/projects/sounds/quickie_in_bed.mp3"),
    Sound("/home/mcboatface/projects/sounds/swedish_jazz.mp3"),
    Sound("/home/mcboatface/projects/sounds/vacuum_cleaner.mp3"),
    Sound("/home/mcboatface/projects/sounds/cockroaches.mp3"),
    Sound("/home/mcboatface/projects/sounds/morse_code.mp3"),
]


play_channel = pygame.mixer.Channel(1)

LAST_SOUND = None # to avoid repeats

INVERTED = False
# this is a flag to track if we need to flip the on/off logic of the button
# because we're using a nonmomentary button, i.e. a switch, but it physically
# looks like a button so it's confusing

WAIT_TIMEOUT = 22
# 1 sec longer than the longest sound we have
# Means if user waits all the way until sound is over, then push button
# it will play a new song. But if they push while sound playing,
# functions as an "off"

def play_random_sound():
    global LAST_SOUND
    sound = random.choice(button_sounds)
    # ensure we never repeat the last sound
    while (sound == LAST_SOUND):
        sound = random.choice(button_sounds)
    play_channel.play(sound)
    play_channel.set_volume(0.1)
    print("played a sound")
    LAST_SOUND = sound

def play_sound_from_press():
    global INVERTED
    global button
    play_random_sound()
    button.wait_for_release(timeout=WAIT_TIMEOUT)
    print("just finished waiting")
    # handle case where user walks away without ever untoggling the button
    if button.is_pressed:
        # means the timeout was hit without releasing
        # now we need to treat presses as if they were releases!
        INVERTED = not INVERTED
        stop()
        set_inverted_handlers(button, play_random_sound, stop)

def play_sound_from_release():
    global INVERTED
    global button
    play_random_sound()
    button.wait_for_press(timeout=WAIT_TIMEOUT)
    if not button.is_pressed:
        # flip again
        INVERTED = not INVERTED
        stop()
        set_normal_handlers(button, play_random_sound, stop)

def set_normal_handlers(button, play_random_sound, stop):
    button.when_pressed = play_sound_from_press
    button.when_released = stop

def set_inverted_handlers(button, play_random_sound, stop):
    button.when_pressed = stop
    button.when_released = play_sound_from_release

if not INVERTED:
    set_normal_handlers(button, play_random_sound, stop)
else:
    set_inverted_handlers(button, play_random_sound, stop)

pause()

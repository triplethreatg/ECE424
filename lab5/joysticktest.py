import pygame
from pygame.locals import *

pygame.init()

num_of_joysticks = pygame.joystick.get_count()
print(num_of_joysticks)

Joystick = pygame.joystick.Joystick(0)
Joystick.init()
print "The Joystick is initialized ", Joystick.get_init()
print "The Joystick id is: ", Joystick.get_id()
print "The Joystick name is: ", Joystick.get_name()
print "The number of buttons on the joystick is: ", Joystick.get_numbuttons()
print range(Joystick.get_numbuttons())

while 1:
	for e in pygame.event.get():
		if e.type == pygame.locals.JOYBUTTONDOWN:
			index = "Input.BUTTON-%d" % e.button
			#index =  % e.button
			print index
			

##for i in range(Joystick.get_numbuttons()):
##	print "button: ", i, " has current state: ", Joystick.get_button(i)
##for i in range(0, Joystick.get_numbuttons()):
##	pygame.event.get()
	
##        print Joystick.get_button(i)
#while((Joystick.get_button(1))==0):
#	pygame.event.get()
#print "button 1 pressed"

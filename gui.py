import pygame as pg
from Sprites import *
pg.init()

screen = pg.display.set_mode([500, 500])

running = True
robot = Robot(screen)
while running:

    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    screen.fill((255, 255, 255))
    robot.draw()
    # pg.draw.circle(screen, (0, 0, 255), (250, 250), 75)

    pg.display.flip()

pg.quit()
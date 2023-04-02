
# Planning
# Dynamic Window Approach (Local Planning) with moving obstacles
# Andrew Davison 2019
import pygame, os, math, time, random, copy
from pygame.locals import *


pygame.init()



# Global Constants and variables
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.1
ROBOTRADIUS = 0.1
WHEELBLOB = 0.04
W = 2 * ROBOTRADIUS # width of robot
SAFEDIST = ROBOTRADIUS      # used in the cost function for avoiding obstacles

MAXVELOCITY = 0.5     #ms^(-1) max speed of each wheel
MAXACCELERATION = 0.4 #ms^(-2) max rate we can change speed of each wheel


BARRIERVELOCITYRANGE = 0.2


# The region we will fill with obstacles
PLAYFIELDCORNERS = (-4.0, -3.0, 4.0, 3.0)

# Timestep delta to run control and simulation at
dt = 0.1
STEPSAHEADTOPLAN = 10
TAU = dt * STEPSAHEADTOPLAN



robots = []


class Robot:
        def __init__(self, x, y, theta):
                self.x = x
                self.y = y
                self.theta = theta
                self.vL = 0.0
                self.vR = 0.0
                self.locationhistory = []
                self.pathstodraw = []

        def draw(self):
                u = u0 + k * self.x
                v = v0 - k * self.y
                pygame.draw.circle(screen, white, (int(u), int(v)), int(k * ROBOTRADIUS), 3)
                # Draw wheels as little blobs so you can see robot orientation
                # left wheel centre 
                wlx = self.x - (W/2.0) * math.sin(self.theta)
                wly = self.y + (W/2.0) * math.cos(self.theta)
                ulx = u0 + k * wlx
                vlx = v0 - k * wly
                pygame.draw.circle(screen, blue, (int(ulx), int(vlx)), int(k * WHEELBLOB))
                # right wheel centre 
                wrx = self.x + (W/2.0) * math.sin(self.theta)
                wry = self.y - (W/2.0) * math.cos(self.theta)
                urx = u0 + k * wrx
                vrx = v0 - k * wry
                pygame.draw.circle(screen, blue, (int(urx), int(vrx)), int(k * WHEELBLOB))


                # Draw paths: little arcs which show the different paths the robot is selecting between
                # A bit complicated so don't worry about the details!
                for path in self.pathstodraw:
                        #if path[0] = 1:    # Pure rotation: nothing to draw
                        if path[0] == 0:    # Straight line
                                straightpath = path[1]
                                linestart = (u0 + k * self.x, v0 - k * self.y)
                                lineend = (u0 + k * (self.x + straightpath * math.cos(self.theta)), v0 - k * (self.y + straightpath * math.sin(self.theta)))
                                pygame.draw.line(screen, (0, 200, 0), linestart, lineend, 1)
                        if path[0] == 2:    # General case: circular arc
                                # path[2] and path[3] are start and stop angles for arc but they need to be in the right order to pass
                                if (path[3] > path[2]):
                                        startangle = path[2]
                                        stopangle = path[3]
                                else:
                                        startangle = path[3]
                                        stopangle = path[2]
                                # Pygame arc doesn't draw properly unless angles are positive
                                if (startangle < 0):
                                        startangle += 2*math.pi
                                        stopangle += 2*math.pi
                                if (path[1][1][0] > 0 and path[1][0][0] > 0 and path[1][1][1] > 1):
                                        #print (path[1], startangle, stopangle)
                                        pygame.draw.arc(screen, (0, 200, 0), path[1], startangle, stopangle, 1)

                # for loc in self.locationhistory:
                #         pygame.draw.circle(screen, grey, (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 0)


# Barrier (obstacle) locations
barriers = []
# barrier contents are (bx, by, visibilitymask)
# Generate some initial random barriers
for i in range(20):
        (bx, by, vx, vy) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]), random.gauss(0.0, BARRIERVELOCITYRANGE), random.gauss(0.0, BARRIERVELOCITYRANGE))
        barrier = [bx, by, vx, vy]
        barriers.append(barrier)

targetindex = random.randint(0,len(barriers))



def printBarriers():
        for (i, barrier) in enumerate(barriers):
                print (i, barrier[0], barrier[1], barrier[2], barrier[3])

def moveBarriers(dt):
        for (i, barrier) in enumerate(barriers):
                barriers[i][0] += barriers[i][2] * dt
                if (barriers[i][0] < PLAYFIELDCORNERS[0]):
                        barriers[i][2] = -barriers[i][2]
                if (barriers[i][0] > PLAYFIELDCORNERS[2]):
                        barriers[i][2] = -barriers[i][2]
                barriers[i][1] += barriers[i][3] * dt
                if (barriers[i][1] < PLAYFIELDCORNERS[1]):
                        barriers[i][3] = -barriers[i][3]
                if (barriers[i][1] > PLAYFIELDCORNERS[3]):
                        barriers[i][3] = -barriers[i][3]
                

        
# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y

# set the width and height of the screen (pixels)
WIDTH = 1500
HEIGHT = 1000


size = [WIDTH, HEIGHT]
black = (20,20,40)
lightblue = (0,120,255)
darkblue = (0,40,160)
red = (255,100,0)
white = (255,255,255)
blue = (0,0,255)
grey = (70,70,70)
green = (0, 255, 0)
# black = (0, 0, 0)
k = 160 # pixels per metre for graphics
idx = 20
# Screen centre will correspond to (x, y) = (0, 0)
u0 = WIDTH / 2
v0 = HEIGHT / 2



X = 1500
Y = 1000

# Initialise Pygame display screen
screen = pygame.display.set_mode(size)
# This makes the normal mouse pointer invisible in graphics window
pygame.mouse.set_visible(0)
pygame.display.set_caption('Show Text')

display_surface = pygame.display.set_mode((X, Y))
font = pygame.font.Font('freesansbold.ttf', 32)
text = font.render('Admit', True, blue, black)
text1 = font.render('Salahuddin', True, blue, black)
# create a rectangular object for the
# text surface object
textRect = text.get_rect()
textRect1 = text1.get_rect()
# set the center of the rectangular object.
textRect.center = (X // 2, Y // 2)
textRect1.center = (X // 2, Y // 1.8)





# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
        # Simple special cases
        # Straight line motion
        if (round (vL,3) == round(vR,3)):
                xnew = x + vL * deltat * math.cos(theta)
                ynew = y + vL * deltat * math.sin(theta)
                thetanew = theta
                path = (0, vL * deltat)   # 0 indicates pure translation
        # Pure rotation motion
        elif (round(vL,3) == -round(vR,3)):
                xnew = x
                ynew = y
                thetanew = theta + ((vR - vL) * deltat / W)
                path = (1, 0) # 1 indicates pure rotation
        else:
                # Rotation and arc angle of general circular motion
                # Using equations given in Lecture 2
                R = W / 2.0 * (vR + vL) / (vR - vL)
                deltatheta = (vR - vL) * deltat / W
                xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
                ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
                thetanew = theta + deltatheta

                # To calculate parameters for arc drawing (complicated Pygame stuff, don't worry)
                # We need centre of circle
                (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
                # Turn this into Rect
                Rabs = abs(R)
                ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
                if (R > 0):
                        start_angle = theta - math.pi/2.0
                else:
                        start_angle = theta + math.pi/2.0
                stop_angle = start_angle + deltatheta
                path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

        return (xnew, ynew, thetanew, path)






# Planning
# We want to find the best benefit where we have a positive component for closeness to target,
# and a negative component for closeness to obstacles, for each of a choice of possible actions
FORWARDWEIGHT = 12
OBSTACLEWEIGHT = 6666

def chooseAction(vL, vR, x, y, theta):
        bestBenefit = -100000
        # Range of possible motions: each of vL and vR could go up or down a bit
        vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
        vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
        pathstodraw = [] # We will store path details here for plotting later
        for vLpossible in vLpossiblearray:
                for vRpossible in vRpossiblearray:
                        # We can only choose an action if it's within velocity limits
                        if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                                # Predict new position in TAU seconds
                                (xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
                                pathstodraw.append(path)
                                # What is the distance to the closest obstacle from this possible position?
                                distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
                                # Calculate how much close we've moved to target location
                                previousTargetDistance = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
                                newTargetDistance = math.sqrt((xpredict - barriers[targetindex][0])**2 + (ypredict - barriers[targetindex][1])**2)
                                distanceForward = previousTargetDistance - newTargetDistance
                                # Alternative: how far have I moved forwards?
                                # distanceForward = xpredict - x
                                # Positive benefit
                                distanceBenefit = FORWARDWEIGHT * distanceForward
                                # Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
                                if (distanceToObstacle < SAFEDIST):
                                        obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                                else:
                                        obstacleCost = 0.0
                                # Total benefit function to optimise
                                benefit = distanceBenefit - obstacleCost
                                #print ("distanceToObstacle", distanceToObstacle, "obstacleCost", obstacleCost, "benefit", benefit)
                                if (benefit > bestBenefit):
                                        vLchosen = vLpossible
                                        vRchosen = vRpossible
                                        bestBenefit = benefit
        return (vLchosen, vRchosen, pathstodraw)




# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
        #print("finding closest obstactle from", x, y, "barriers", len(barriers))
        closestdist = 100000.0  
        # Calculate distance to closest obstacle
        for (i,barrier) in enumerate(barriers):
                if (i != targetindex):
                        dx = barrier[0] - x
                        dy = barrier[1] - y
                        d = math.sqrt(dx**2 + dy**2)
                        # Distance between closest touching point of circular robot and circular barrier
                        dist = d - BARRIERRADIUS -      ROBOTRADIUS
                        if (dist < closestdist):
                                closestdist = dist
                                #print ("closest barrier number", i, "closestdist =", closestdist)
        return closestdist

# Draw the barriers on the screen
def drawBarriers(barriers):
        for (i,barrier) in enumerate (barriers):
                if (i == targetindex):
                        bcol = green
                else:
                        bcol = lightblue

                pygame.draw.circle(screen, bcol, (int(u0 + k * barrier[0]), int(v0 - k * barrier[1])), int(k * BARRIERRADIUS), 0)







# Initialise Robot
robots = []
for i in range(5):
        robots.append(Robot(PLAYFIELDCORNERS[0] - 0.5, -2.0 + 0.8 * i, 0.0))





# Main loop
while(1):
        display_surface.fill(white)
        display_surface.blit(text, textRect)
        display_surface.blit(text1, textRect1)
        for event in pygame.event.get():
 
        # if event object type is QUIT
        # then quitting the pygame
        # and program both.
                if event.type == pygame.QUIT:
        
                # deactivates the pygame library
                        pygame.quit()
                
                        # quit the program.
                        quit()
        
        # Draws the surface object to the screen.
        pygame.display.update()

        # Eventlist = pygame.event.get()
                

        
        # Copy of barriers so we can predict their positions
        barrierscopy = copy.deepcopy(barriers)

        for i in range(STEPSAHEADTOPLAN):
                moveBarriers(dt)

        #print ("barriers", len(barriers))

        # First robot plans without taking account of other robots
        # Then becomes a barrier for subsequent robots

        robotslistrandomised = robots.copy()
        random.shuffle(robotslistrandomised) 
        
        for robot in robotslistrandomised:

                # For display of trail
                robot.locationhistory.append((robot.x, robot.y))
                (robot.vL, robot.vR, robot.pathstodraw) = chooseAction(robot.vL, robot.vR, robot.x, robot.y, robot.theta) 


                (predx, predy, predtheta, tmppath) = predictPosition(robot.vL, robot.vR, robot.x, robot.y, robot.theta, TAU)

                
                barrier = [predx, predy, 0.0, 0.0]
                barriers.append(barrier)


        
        # Move barriers back
        barriers = copy.deepcopy(barrierscopy)


        
        # Planning is finished; now do graphics
        screen.fill(black)
        drawBarriers(barriers)

        
        # Draw robots
        for robot in robots:
                robot.draw()


        # Update display
        pygame.display.flip()

        # Actually now move robots based on chosen vL and vR
        for robot in robots:
                (robot.x, robot.y, robot.theta, tmppath) = predictPosition(robot.vL, robot.vR, robot.x, robot.y, robot.theta, dt)


        moveBarriers(dt)

        
        # Wraparound: check if robot has reached target; if so reset it to the other side, randomise
        # target position and add some more barriers to go again
        mindist = 1000000.0
        for robot in robots:
                disttotarget = math.sqrt((robot.x - barriers[targetindex][0])**2 + (robot.y - barriers[targetindex][1])**2)
                if (disttotarget < mindist):
                        mindist = disttotarget
                
        if (mindist < (BARRIERRADIUS + ROBOTRADIUS)):
                # Add new barriers
                for i in range(2):
                        (bx, by) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]))
                        (bx, by, vx, vy) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]), random.uniform(-BARRIERVELOCITYRANGE, BARRIERVELOCITYRANGE), random.uniform(-BARRIERVELOCITYRANGE, BARRIERVELOCITYRANGE))
                        barrier = [bx, by, vx, vy]
                        barriers.append(barrier)
                targetindex = random.randint(0,len(barriers)-1)

                
                # Reset trail
                for robot in robots:
                        robot.locationhistory = []

                
        # Sleeping dt here runs simulation in real-time
        time.sleep(dt / 5)
        #time.sleep(1.0)
        
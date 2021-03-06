import pygame
import numpy as np

CONTROL_ACC = set(['FWD', 'BACK'])
CONTROL_ANG = set(['LEFT', 'RIGHT'])

def update_vel(vel, heading, force, mass, drag=0.1):
    """
    Returns a new velocity vector...
    """
    acc = force/mass
    new_vel = np.array(vel)
    new_vel[0] = vel[0] + acc*np.cos(heading) - drag*vel[0]
    # use -sin because in pygame, positive y is down
    new_vel[1] = vel[1] + acc*-np.sin(heading) - drag*vel[1]
    return new_vel

def update_pos(pos, heading, vel):
    """
    Returns a new position vector...
    """
    new_pos = np.array(pos)
    new_pos[0] = pos[0] + vel[0]
    new_pos[1] = pos[1] + vel[1]
    return new_pos

def minimize_angle(ang):
    """
    Compress the value of an agle to be between pi and -pi
    """
    ang = ang%(2*np.pi)
    if ang > np.pi:
        ang = 2*np.pi - ang
    if ang < -np.pi:
        ang = 2*np.pi + ang
    return ang%(2*np.pi)


class Vehicle(object):

    def __init__(self, mass = 25.0, force=1.0, ang=0.001,
            init_pos = np.array([0.0, 0.0]),
            noise = 0.0, drag=0.3, main=False):
        """
        Defines a new vehicle.
        noise is the variance of the noise distribution.
        drag is some drag coefficient... (not actually a physical parameter)
        """
        self.mass = mass
        self.force = force
        # ang = angular velocity
        self.ang = ang
        self.pos = init_pos
        # angle heading in radians (?)
        self.heading = 0.0
        self.vel = np.array([0.0, 0.0])
        # angular velocity
        self.max_vel = 10.0
        self.noise = noise
        self.drag = drag
        self.surface = self.create_display_surface(main)
        self.is_main = main

    def state(self):
        """
        Returns the state of the vehicle:
            x-position, y-position, heading, x-velocity, y-velocity
        """
        return np.array([self.pos[0], self.pos[1],
                minimize_angle(self.heading), self.vel[0],
                self.vel[1]])

    def update_hypothetical(self, control):
        """
        Does a hypothetical update without actually changing the vehicle's
        state.
        """

    def update(self, control):
        """
        Updates the vehicle state
        """
        if 'FWD' in control:
            self.vel = update_vel(self.vel, self.heading, self.force,
                    self.mass, self.drag)
        elif 'BACK' in control:
            self.vel = update_vel(self.vel, self.heading, -self.force,
                    self.mass, self.drag)
        else:
            self.vel = update_vel(self.vel, self.heading, 0.0,
                    self.mass, self.drag)
        if 'LEFT' in control:
            self.heading += self.ang
        elif 'RIGHT' in control:
            self.heading -= self.ang
        self.pos = update_pos(self.pos, self.heading, self.vel)

    def create_display_surface(self, main=False):
        """
        Creates a pygame surface for displaying the vehicle.
        """
        vehicle1_surface = pygame.Surface((50, 50))
        pygame.draw.ellipse(vehicle1_surface, (255,255,255),(0,0,50,50))
        if main:
            pygame.draw.ellipse(vehicle1_surface, (255,0,0),(0,0,50,50))
        pygame.draw.line(vehicle1_surface, (0,0,255),(25,25),(50,25))
        self.surface = vehicle1_surface
        return vehicle1_surface

    def draw(self, surface, main_pos = np.array([0,0])):
        """
        Draws the vehicle on a surface.
        main_pos: the 'center' position of the surface. For example, if the
        surface is always centered at a specific object, then it's the position
        of that object.
        """
        width, height = surface.get_size()
        rect = self.surface.get_rect()
        angle = np.degrees(self.heading)
        new_img = pygame.transform.rotate(self.surface, angle)
        rot_rect = rect.copy()
        rot_rect.center = new_img.get_rect().center
        # center it...
        new_img = new_img.subsurface(rot_rect).copy()
        new_pos = self.pos - main_pos + np.array([width/2, height/2])
        # TODO: make rotation work better...
        surface.blit(new_img, new_pos)

    def detect_collision(self, other):
        """
        Detects collision with another vehicle.
        """
        # arbitary constant of 50 for the radius/distance
        if np.sqrt(np.dot(self.pos-other.pos, self.pos-other.pos)) <= 50:
            return True
        return False

def main():
    vehicle1 = Vehicle()
    vehicle1.pos = np.array([300.0,300.0])
    screen = pygame.display.set_mode((600,600))
    while 1:
        control = []
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        #print [i for i in keys if i>0]
        if keys[pygame.K_UP]:
            control.append('FWD')
        if keys[pygame.K_LEFT]:
            control.append('LEFT')
        if keys[pygame.K_DOWN]:
            control.append('BACK')
        if keys[pygame.K_RIGHT]:
            control.append('RIGHT')
        vehicle1.update(control)
        #print control
        #print vehicle1.state()
        screen.fill((0,0,0))
        vehicle1.draw(screen)

if __name__ == '__main__':
    main()

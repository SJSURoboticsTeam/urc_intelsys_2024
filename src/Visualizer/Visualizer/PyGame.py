from dataclasses import dataclass, field
# Example file showing a circle moving on screen
import pygame
import math
import numpy as np

rt3 = math.sqrt(3)

pygame.font.init()

@dataclass
class HexagonCell:
    position: pygame.Vector2
    edgeLen: int
    coordMap: dict
    coord: pygame.Vector2 = pygame.Vector2(0,0)
    color: str = 'red'
    def get_points(self):
        x, y = self.position
        l = self.edgeLen
        return [
            (x+l, y),
            (x+l/2, y+rt3*l/2),
            (x-l/2, y+rt3*l/2),
            (x-l, y),
            (x-l/2, y-rt3*l/2),
            (x+l/2, y-rt3*l/2),
        ]
    def __post_init__(self):
        self.coordMap.setdefault(self.coord.x, {})[self.coord.y]=self
        # print(self.coord)
        pass


@dataclass
class HexagonStack:
    position: pygame.Vector2
    edgeLen: int
    stackSize: int
    coordMap: dict
    spacerFrac: float = 0.2
    xcoord: int = 0
    got_hexagons: bool = False
    def _get_hexagons(self):
        arr = np.arange(self.stackSize) * (rt3+self.spacerFrac)*self.edgeLen
        arr -= arr[-1]/2
        mid = arr.shape[0]//2
        return [HexagonCell(
            position=self.position + (0, y),
            edgeLen=self.edgeLen,
            coordMap=self.coordMap,
            coord=pygame.Vector2(self.xcoord, 2*(i-mid)+mid%2)
            ) for i, y in enumerate(arr)]
    def get_hexagons(self):
        if self.got_hexagons:
            return self.hexagons 
        else:
            self.hexagons = self._get_hexagons()
            self.got_hexagons = True
            return self.hexagons


@dataclass
class HexGrid:
    columns: int
    """The number of columns
    """
    cellStack: int
    """The number of cells stacked to make a column
    """
    edgeLen: int
    """Edge length of every cell int eh hexagon
    """
    position: pygame.Vector2
    """Centerd position of the grid on the screen
    """
    coordMap: dict = field(default_factory=dict)
    spacerFrac: float = 0.2
    got_stacks: bool = False
    
    def _get_stacks(self):
        columns = np.arange(self.columns) * (1.5+rt3*self.spacerFrac/2)*self.edgeLen
        columns -= columns[-1]/2
        stacks = np.arange(self.columns)%2
        stacks += self.cellStack
        mid = columns.shape[0]//2
        return [HexagonStack(
            position=self.position+(x,0), 
            edgeLen=self.edgeLen, 
            stackSize=stack, 
            spacerFrac=self.spacerFrac,
            coordMap=self.coordMap,
            xcoord=i-mid
            ) for i, (x,stack) in enumerate(zip(columns, stacks))]
    def get_stacks(self):
        if self.got_stacks:
            return self.stacks 
        else:
            self.stacks = self._get_stacks()
            self.got_stacks = True
            return self.stacks

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0


player_pos = pygame.Vector2(screen.get_width() / 2, screen.get_height() / 2)
hexedge = 60
font = pygame.font.Font(size=int(hexedge*0.8))
hexGrid = HexGrid(5, 5, hexedge, player_pos, {})

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("orange")

    # pygame.draw.circle(screen, "red", player_pos, 40)
    # pygame.draw.polygon(screen, 'red', hexpoints(player_pos, hexedge))
    # pygame.draw.polygon(screen, 'red', hexagon.get_points())
    def draw_hexagon(hexagon: HexagonCell):
        pygame.draw.polygon(screen, hexagon.color, hexagon.get_points())
        text = font.render(str(hexagon.coord), True, 'green')
        rect = text.get_rect()
        rect.center = hexagon.position
        screen.blit(text, rect)
    
    # try:
    #     hexGrid.coordMap[0.0][0.0].color='black'
    # except KeyError:
    #     pass
    [
        # [pygame.draw.polygon(screen, 'red', h.get_points(), 3) for h in hexStack.get_hexagons()]
        [draw_hexagon(h) for h in hexStack.get_hexagons()]
        for hexStack in hexGrid.get_stacks()
    ]

    keys = pygame.key.get_pressed()
    if keys[pygame.K_o]:
        hexGrid.coordMap[0.0][0.0].color='black'
    if keys[pygame.K_p]:
        hexGrid.coordMap[0.0][0.0].color='red'

    

    # if keys[pygame.K_w]:
    #     player_pos.y -= 300 * dt
    # if keys[pygame.K_s]:
    #     player_pos.y += 300 * dt
    # if keys[pygame.K_a]:
    #     player_pos.x -= 300 * dt
    # if keys[pygame.K_d]:
    #     player_pos.x += 300 * dt

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()

import numpy as np
from base_letter import Letter
class A(Letter):
    '''
                2
              1 3 4
            0       5      
    '''
    def set_positions(self, nPts):
        if nPts is None:
            nPts = 6
        elif nPts < 6:
            raise ValueError("Number of points must be at least 6 for 'A'")
        elif nPts > 6:
            print("[WARN] Number of points is more than 6 for 'A', not implemented yet. Defaulting to 6.")
            nPts = 6
        horiz_divs = np.linspace(0, self.width, 5)
        self.positions = np.zeros((nPts, 3))
        self.positions[0] = [0, 0, 0] 
        self.positions[1] = [horiz_divs[1], self.height/2, 0] 
        self.positions[2] = [horiz_divs[2], self.height, 0] 
        self.positions[3] = [horiz_divs[2], self.height/2, 0] 
        self.positions[4] = [horiz_divs[3], self.height/2, 0] 
        self.positions[5] = [self.width, 0, 0] 

    def set_width(self, width):
        if width is None:
            self.width = self.height
        else:
            self.width = width

class O(Letter):
    '''    
           3
        2     4
        1     5
        0     6 
           7  
    '''
    def set_positions(self, nPts):
        if nPts is None:
            nPts = 8
        elif nPts < 8:
            raise ValueError("Number of points must be at least 8 for 'O'")

        self.positions = np.zeros((nPts, 3))
        theta = np.linspace(0, 2 * np.pi, nPts, endpoint=False)
        self.positions[:, 0] = self.width / 2 * (1 + np.cos(theta))  # x coordinates
        self.positions[:, 1] = self.height / 2 * (1 + np.sin(theta)) # y coordinates

    def set_width(self, width):
        if width is None:
            self.width = 0.7*self.height
        else:
            self.width = width

class C(Letter):
    '''    
           3
        2     
        1     
        0     
           4  
    '''
    def set_positions(self, nPts):
        if nPts is None:
            nPts = 5
        elif nPts < 5:
            raise ValueError("Number of points must be at least 5 for 'C'")
        elif nPts > 5:
            print("[WARN] Number of points is more than 5 for 'C', not implemented yet. Defaulting to 5.")
            nPts = 5

        self.positions = np.zeros((nPts, 3))
        theta = np.linspace(np.pi / 2, 3 * np.pi / 2, nPts)
        self.positions[:, 0] = self.width / 2 * (1 + np.cos(theta))  # x coordinates
        self.positions[:, 1] = self.height / 2 * (1 + np.sin(theta)) # y coordinates

    def set_width(self, width):
        if width is None:
            self.width = 1.5*self.height
        else:
            self.width = width

class R(Letter):
    '''    
        3  4  5
        
        2   6
        1     7
        0        8 
    '''
    def set_positions(self, nPts):
        if nPts is None:
            nPts = 9
        elif nPts < 9:
            raise ValueError("Number of points must be at least 9 for 'R'")
        elif nPts > 9:
            print("[WARN] Number of points is more than 9 for 'R', not implemented yet. Defaulting to 9.")
            nPts = 9

        self.positions = np.zeros((nPts, 3))
        vertical_divs = np.linspace(0, self.height, 5)
        horiz_divs = np.linspace(0, self.width, 7)
        self.positions[0] = [0, 0, 0]
        self.positions[1] = [0, vertical_divs[1], 0]
        self.positions[2] = [0, vertical_divs[2], 0]
        self.positions[3] = [0, vertical_divs[4], 0]
        self.positions[4] = [horiz_divs[2], self.height, 0]
        self.positions[5] = [horiz_divs[4], self.height, 0]
        self.positions[6] = [horiz_divs[3], vertical_divs[2], 0]
        self.positions[7] = [horiz_divs[4], vertical_divs[1], 0]
        self.positions[8] = [self.width, 0, 0]

    def set_width(self, width):
        if width is None:
            self.width = 0.75*self.height
        else:
            self.width = width

class L(Letter):
    '''    
        2
        1
        0 3
    '''
    def set_positions(self, nPts):
        if nPts is None:
            nPts = 4
        elif nPts < 4:
            raise ValueError("Number of points must be at least 4 for 'L'")
        elif nPts > 4:
            print("[WARN] Number of points is more than 4 for 'L', not implemented yet. Defaulting to 4.")
            nPts = 4

        self.positions = np.zeros((nPts, 3))
        self.positions[0] = [0, 0, 0]  
        self.positions[1] = [0, self.height/2, 0]  
        self.positions[2] = [0, self.height, 0]  
        self.positions[3] = [self.width, 0, 0]  

    def set_width(self, width):
        if width is None:
            self.width = 0.33*self.height
        else:
            self.width = width
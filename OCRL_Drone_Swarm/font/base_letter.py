from matplotlib import pyplot as plt
import numpy as np
from abc import ABC, abstractmethod
class Letter(ABC):
    """
    Abstract base class representing a letter with geometric attributes.

    Attributes:
        height (float): Height of the letter.
        width (float): Width of the letter, calculated based on the letter type.
        positions (np.ndarray): Array of positions defining the letter's shape.
        start (np.ndarray): Start coordinates of the letter.
        end (np.ndarray): End coordinates of the letter.

    Methods:
        set_positions(nPts): Abstract method to set the positions of the letter.
        set_width(width): Abstract method to set the width of the letter.
        plot2D(title): Plots the letter in a 2D space.
    """

    def __init__(self, height, width=None, nPts=None, viz=False):
        """
        Initializes the Letter object with specified height, optional width, and point count.

        Parameters:
            height (float): Height of the letter.
            width (float, optional): Width of the letter.
            nPts (int, optional): Number of points to define the letter shape.
            viz (bool, optional): If True, visualizes the letter upon creation.
        """
        self.height = height
        self.set_width(width)
    
        self.positions = None
        self.set_positions(nPts)

        self.start = np.min(self.positions, axis=0)
        self.end = np.max(self.positions, axis=0)

        if viz:
            self.plot2D(title=f"Letter {self.__class__.__name__} defined with {nPts} points.")
    
    @abstractmethod
    def set_positions(self, nPts):
        """
        Abstract method to be implemented by subclasses to define the letter's geometric positions.

        Parameters:
            nPts (int): Number of points to define the letter.
        """
        pass
    
    @abstractmethod
    def set_width(self, width):
        """
        Abstract method to be implemented by subclasses to set the letter's width.

        Parameters:
            width (float): Width of the letter.
        """
        pass

    def plot2D(self, title=""):
        """
        Plots the letter in a 2D plot using matplotlib.

        Parameters:
            title (str, optional): Title of the plot.
        """
        plt.scatter(self.positions[:, 0], self.positions[:, 1])
        plt.axis('equal')  
        plt.grid(True) 
        plt.title(title)
        plt.show()

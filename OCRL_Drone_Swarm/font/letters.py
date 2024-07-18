from matplotlib import pyplot as plt
import numpy as np
from letter_definitions import * 

class Letters:
    """
    A class to manage and visualize a string of geometric letters.

    Attributes:
        letters (list): List of Letter instances representing the letters in the string.
        positions (np.ndarray): Aggregated positions of all letters in the string.

    Methods:
        set_positions(spacing, start): Sets positions for all letters considering spacing.
        plot2D(title): Plots all letters in the string in a 2D space.
    """

    letter_classes = {'A': A, 'C': C,  'L': L, 'O': O, 'R': R,}
    
    def __init__(self, text, height, spacing=2, start=[0,0,0], viz=False):
        """
        Initializes a string of letters with given height and spacing.

        Parameters:
            text (str): String of characters.
            height (float): Uniform height for all letters.
            spacing (float, optional): Spacing between letters. Defaults to 2.
            viz (bool, optional): If True, visualizes the string upon creation.
        """
        self.letters = [self.letter_classes[letter](height) for letter in text]
        self.set_positions(spacing, start)
        if viz:
            self.plot2D(title=text)

    def set_positions(self, spacing, start):
        """
        Sets the positions of each letter in the string based on the specified spacing.

        Parameters:
            spacing (float): Spacing between adjacent letters.
            start (list of float): Starting position for the first letter.
        """
        self.positions = np.empty((0, 3))
        current = start
        for letter in self.letters:
            local_positions = letter.positions
            self.positions = np.vstack((self.positions, current + local_positions))
            current[0] = current[0] + letter.end[0] + spacing 

    def plot2D(self, title=""):
        """
        Plots a 2D visualization of the letter string.

        Parameters:
            title (str, optional): Title of the plot.
        """
        plt.scatter(self.positions[:, 0], self.positions[:, 1])
        plt.axis('equal')  
        plt.grid(True) 
        plt.title(title)
        plt.show()

if __name__ == '__main__':
    Letters("OCRL", 2, viz=True)
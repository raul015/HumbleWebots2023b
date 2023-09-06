import pandas as pd
from matplotlib import pyplot as plt
import csv

def main():
    
    x_noReal_on = []
    y_noReal_on = []
    
    x_Real_on = []
    y_Real_on = []
    
    x_noReal_off = []
    y_noReal_off = []
    
    x_Real_off = []
    y_Real_off = []
    
    with open('src/test_slittamento/test_slittamento/FluidoOn_NoReal.csv','r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            x_noReal_on.append(float(row[0]))
            y_noReal_on.append(float(row[1]))
    
    
    with open('src/test_slittamento/test_slittamento/FluidoOn_Real.csv','r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            x_Real_on.append(float(row[0]))
            y_Real_on.append(float(row[1]))
            
            
    with open('src/test_slittamento/test_slittamento/FluidoOff_Real.csv','r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            x_Real_off.append(float(row[0]))
            y_Real_off.append(float(row[1]))
    
    with open('src/test_slittamento/test_slittamento/FluidoOff_NoReal.csv','r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            x_noReal_off .append(float(row[0]))
            y_noReal_off .append(float(row[1]))
            
            
    plt.plot(x_noReal_on, y_noReal_on, color = 'g',
            label = "odometria_con")
    
    plt.plot(x_Real_on, y_Real_on, color = 'r', 
             label = "Real_con")
    
    plt.plot(x_noReal_off, y_noReal_off, color = 'b',
            label = "odometria_senza")
    plt.plot(x_Real_off, y_Real_off, color = 'y',
             label = "Real_senza")
    
    plt.xticks(rotation = 25)
    plt.xlabel('Posizione x')
    plt.ylabel('Posizione y')
    plt.title('Mappa delle posizioni x,y', fontsize = 20)
    plt.grid()
    plt.legend()
    plt.show()
    


if __name__ == "__main__":
    main()

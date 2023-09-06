import pandas as pd
from matplotlib import pyplot as plt
import csv

def main():
    
    
    x_1 = []
    y_1 = []
    
    x_2 = []
    y_2 = []
    
    with open('src/olio_test/olio_test/prova.csv','r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            x_1.append(float(row[0]))
            y_1.append(float(row[1]))
    
    with open('src/olio_test/olio_test/prova1.csv','r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            x_2.append(float(row[0]))
            y_2.append(float(row[1]))
            
    plt.plot(x_1, y_1, color = 'g', linestyle = 'dashed',
            marker = 'o',label = "odometria")
    plt.plot(x_2, y_2, color = 'r', linestyle = 'dashed',
             marker = 'o',label = "Real")
    
    plt.xticks(rotation = 25)
    plt.xlabel('Posizione x')
    plt.ylabel('Posizione y')
    plt.title('Mappa delle posizioni x,y', fontsize = 20)
    plt.grid()
    plt.legend()
    plt.show()
    
    

    
    # plt.plot(x_2, y_2, color = 'g', linestyle = 'dashed',
    #         marker = 'o',label = "Weather Data")
    
    # plt.xticks(rotation = 25)
    # plt.xlabel('Posizione x')
    # plt.ylabel('Posizione y')
    # plt.title('Mappa delle posizioni x,y', fontsize = 20)
    # plt.grid()
    # plt.legend()
    # plt.show()

if __name__ == "__main__":
    main()

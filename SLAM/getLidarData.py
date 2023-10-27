import pandas as pd
import numpy as np



def getDataTxt(data_set=100, num_rev=1):
    data_per_rev = 3290
    df = pd.read_csv('./angles.txt', sep=' ', names=['angle'], nrows=data_per_rev*num_rev,
                     skiprows=data_per_rev*data_set)
    df2 = pd.read_csv('./distances.txt', sep=' ', names=['distance'], nrows=data_per_rev*num_rev,
                      skiprows=data_per_rev*data_set)

    data = pd.concat([df, df2], axis=1)
    data['angle'].astype('float64')

    data['angle'].astype('float64')

    data = data[(data.distance < 10000) & (data.distance > 40)]

    data['x'] = -(data['distance'] * np.cos(np.radians(data['angle']))).to_numpy()
    data['y'] = (data['distance'] * np.sin(np.radians(data['angle']))).to_numpy()
    data['z'] = (data['distance'] * 0).to_numpy()


    datX = pd.concat([data['x'], data['y'], data['z']], axis=1)
    X = datX.to_numpy()
    # print(X)
    i = 0
    filename = 'data/cloud' + str(i) + '.xyz'
    i += 1
    np.savetxt(filename, X, delimiter='\t')


def getDataXYZ(data_set=100, num_rev=1, num_sets=10):
    data_per_rev = 3630
    data = pd.read_csv('./scanCloud.xyz', sep=' ', names=['x', 'y', 'z'], nrows=data_per_rev*num_rev,
                       skiprows=data_per_rev*data_set)
    data = data[((data.x.abs() < 8000) & (data.y.abs() < 8000))]

    X = data.to_numpy()

    for i in range(num_sets):
        filename = 'data/cloud' + str(i) + '.xyz'
        np.savetxt(filename, X, delimiter='\t')

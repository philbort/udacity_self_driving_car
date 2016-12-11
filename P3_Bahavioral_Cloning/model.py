import pandas as pd
import numpy as np
import matplotlib.image as mpimg
import cv2
import json
from sklearn.model_selection import train_test_split
from keras.models import Sequential, model_from_json
from keras.layers import Dense, Dropout, Flatten, Lambda, ELU, Convolution2D


# Import data from the csv file
def import_data(csv_file):
    data = pd.read_csv(csv_file, header = None)
    data.columns = ['center', 'left', 'right', 'steering', 'throttle', 'brake', 'speed']
    left_data = data[['left', 'steering']]
    left_data.columns = ['image', 'steering']
    center_data = data[['center', 'steering']]
    center_data.columns = ['image', 'steering']
    right_data = data[['right', 'steering']]
    right_data.columns = ['image', 'steering']
    frames = [left_data, center_data, right_data]
    data = pd.concat(frames, axis = 0, ignore_index = True)
    X_train = data.iloc[:,0].values
    y_train = data.iloc[:,1].values.astype(np.float32)
    return X_train, y_train

# Load the image
def load_image(path):
    width, height = 200, 66
    path.replace(' ','')
    image = mpimg.imread(path)
    image = image[int(image.shape[0]/3):image.shape[0], :]
    image = cv2.resize(image, (width, height))
    image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
    return image

# Image generator
def image_generator(x, y, batch_size = 128):
    start = 0
    features = np.ndarray(shape = (batch_size, 66, 200, 3))
    labels = np.ndarray(shape = (batch_size,))
    while True:
        for i in range(start, start + batch_size):
            i = i % len(x)
            path = x[i]
            label = y[i]
            if path.startswith(' '):
                path = path[1:]
            features[i%batch_size] = load_image(path)
            labels[i%batch_size] = label

        start = start + batch_size
        yield (features, labels)

# Define the keras model from comma.ai
def get_model():
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1.,
              input_shape=(66, 200, 3),
              output_shape=(66, 200, 3)))
    model.add(Convolution2D(16, 8, 8, subsample=(4, 4), border_mode="same"))
    model.add(ELU())
    model.add(Convolution2D(32, 5, 5, subsample=(2, 2), border_mode="same"))
    model.add(ELU())
    model.add(Convolution2D(64, 5, 5, subsample=(2, 2), border_mode="same"))
    model.add(Flatten())
    model.add(Dropout(.2))
    model.add(ELU())
    model.add(Dense(512))
    model.add(Dropout(.5))
    model.add(ELU())
    model.add(Dense(1))
    model.compile(optimizer="adam", loss="mse", metrics = ['accuracy'])
    model.summary()
    return model


if __name__ == "__main__":

    csv_file = 'driving_log.csv'    
    modelJSON = 'model.json'
    modelWeights = 'model.h5'

    X_train, y_train = import_data('driving_log.csv')
    print('%s imported' %csv_file)

    X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.2)
    batch_size = 128
    train_size = y_train.shape[0]
    val_size = y_val.shape[0]   
    print('Number of training data: %i' %train_size)
    print('Number of validation data: %i' %val_size)
    print('Batch size: %i' %batch_size)
    model = get_model()
    model.fit_generator(image_generator(X_train, y_train, batch_size),
                        samples_per_epoch = train_size - train_size%batch_size,
                        nb_epoch = 5,
                        validation_data = image_generator(X_val, y_val, val_size),
                        nb_val_samples = val_size)

    json_string = model.to_json()
    with open(modelJSON,'w' ) as f:
        json.dump(json_string, f)
    model.save_weights(modelWeights)
    print('Model saved at %s, weights saved at %s' %(modelJSON, modelWeights))
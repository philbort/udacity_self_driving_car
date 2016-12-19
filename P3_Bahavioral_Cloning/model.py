import argparse
import os
import pandas as pd
import numpy as np
import cv2
import json
from random import randint
from sklearn.model_selection import train_test_split
from keras.models import Sequential, model_from_json
from keras.layers import Dense, Dropout, Flatten, Lambda, ELU, Convolution2D, MaxPooling2D
from keras.optimizers import Adam

# Import data from the csv file
def import_data(csv_file, side_image = True):
    data = pd.read_csv(csv_file)#, header = None)
    data.columns = ['center', 'left', 'right', 'steering', 'throttle', 'brake', 'speed']
    # If we use the side images
    if side_image is True:
        # Left Data
        left_data = data[['left', 'steering']]
        left_data.columns = ['image', 'steering']
        # Only save "turn right" images from left camera
        left_data = left_data[(left_data.steering > 0.0)]
        # Center Data
        center_data = data[['center', 'steering']]
        center_data.columns = ['image', 'steering']
        # Right Data
        right_data = data[['right', 'steering']]
        right_data.columns = ['image', 'steering']
        # Only save "turn left" images from right camera
        right_data = right_data[(right_data.steering < 0.0)]
        # Frame
        frames = [left_data, center_data, right_data]
        # Combine data
        data = pd.concat(frames, axis = 0, ignore_index = True)
    # If we only use center images
    else:
        data = data.drop(['left', 'right', 'throttle', 'brake', 'speed'], axis = 1)
    # Get the training data
    X_train = data.iloc[:,0].values
    y_train = data.iloc[:,1].values.astype(np.float32)
    return X_train, y_train

# Load the image
def load_image(path):
    # Image size (ref: Nvidia paper)
    width, height = 200, 66
    image = cv2.imread(path)
    # Crop the sky and the front of the car
    image = image[60:image.shape[0], :]
    # Resize image
    image = cv2.resize(image, (width, height))
    # Convert color from BGR to YUV (ref: Nvidia paper)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    return image

# Image generator called by keras fit_generator to save memory
def image_generator(x, y, batch_size = 128):

    # Initialization
    epoch_size = len(x) - len(x)%batch_size
    batch_per_epoch = epoch_size/batch_size
    batch_cnt = 0
    start = 0
    features = np.ndarray(shape = (batch_size, 66, 200, 3))
    labels = np.ndarray(shape = (batch_size,))
    while True:
        # If we ran a complete epoch
        if( (batch_cnt % batch_per_epoch) == 0):
            # Shuffle the data
            ind = np.arange(0, epoch_size)
            np.random.shuffle(ind)
            x = x[ind]
            y = y[ind]
            start = 0
        for i in range(start, start + batch_size):
            path = x[i]
            label = y[i]
            # Increase the steering angle for side images to mimick turns
            if ('right' in path):
                label = label - 0.1
            elif ('left' in path):
                label = label + 0.1 
            # Some path starts with a space, no idea why
            if path.startswith(' '):
                path = path[1:]
            features[i%batch_size] = load_image(path)
            labels[i%batch_size] = label
        # Update batch count
        batch_cnt += 1
        start = start + batch_size
        yield (features, labels)

# Load previous model for tuning
def load_model(model_file, model_weights):
    with open(model_file, 'r') as f:
        model = model_from_json(json.load(f))    
    adam = Adam(lr = 0.00005)
    model.compile(optimizer=adam, loss="mse", metrics = ['accuracy'])
    model.load_weights(model_weights)
    print("Load model from %s with weights %s" %(model_file, model_weights))
    return model

# Define the keras model (ref: Nvidia paper)
def get_model():
    pool_size = (2, 2)
    stride = (2, 2)
    dropout = 0.2
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1.,
              input_shape=(66, 200, 3),
              output_shape=(66, 200, 3)))
    model.add(Convolution2D(5, 5, 24, subsample=stride, border_mode="same"))
    model.add(MaxPooling2D(pool_size = pool_size))
    model.add(ELU())
    model.add(Convolution2D(5, 5, 36, subsample=stride, border_mode="same"))
    model.add(ELU())
    model.add(Convolution2D(5, 5, 48, subsample=stride, border_mode="same"))
    model.add(ELU())
    model.add(Convolution2D(3, 3, 64, subsample=stride, border_mode="same"))
    model.add(ELU())
    model.add(Convolution2D(3, 3, 64, subsample=stride, border_mode="same"))
    model.add(Flatten())
    model.add(Dropout(.2))
    model.add(ELU())
    model.add(Dense(1164))
    model.add(Dropout(dropout))
    model.add(ELU())
    model.add(Dense(100))
    model.add(Dropout(dropout))
    model.add(ELU())
    model.add(Dense(50))
    model.add(Dropout(dropout))
    model.add(ELU())
    model.add(Dense(10))
    model.add(Dropout(dropout))
    model.add(ELU())
    model.add(Dense(1))
    adam = Adam(lr = 0.00001)
    model.compile(optimizer=adam, loss="mse", metrics = ['accuracy'])
    return model

if __name__ == "__main__":

    # Parse the input argument
    parser = argparse.ArgumentParser(description='Steering angle model trainer')
    parser.add_argument('-nb_epoch', type=int, default=5, help='Training epoch number')
    parser.add_argument('-batch_size', type=int, default=128, help='Training batch size')
    parser.add_argument('-side_image', dest='side_image', action='store_true')
    parser.add_argument('-no_side_image', dest='side_image', action='store_false')
    parser.set_defaults(side_image=True)
    parser.add_argument('-csv_file', type=str, default="driving_log.csv", help='Training data file')
    parser.add_argument('-tuning', type=bool, default=False, help='Tune from old model or not')
    parser.add_argument('-modelJSON', type=str, default="model.json", help='Model destination')
    parser.add_argument('-modelWeights', type=str, default="model.h5", help='Model weights destination')
    args = parser.parse_args()

    # Import the training data
    X_train, y_train = import_data(args.csv_file, args.side_image)
    print('%s imported' %args.csv_file)
    print('Train with side images? %r'  %args.side_image)

    # Split the data (not really neccesary in this case, hence the small test_size)
    X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.01)
    train_size = y_train.shape[0]
    val_size = y_val.shape[0]   
    print('Number of training data: %i' %train_size)
    print('Number of validation data: %i' %val_size)
    print('Batch size: %i' %args.batch_size)

    # Train the model
    if args.tuning is True:
        model = load_model(args.modelJSON, args.modelWeights)
    else:
        model = get_model()
    model.summary()
    model.fit_generator(image_generator(X_train, y_train, args.batch_size),
                        samples_per_epoch = train_size - train_size%args.batch_size,
                        nb_epoch = args.nb_epoch,
                        validation_data = image_generator(X_val, y_val, val_size),
                        nb_val_samples = val_size)

    # Remove old model if exists
    if os.path.isfile(args.modelJSON):
        os.remove(args.modelJSON)
    if os.path.isfile(args.modelWeights):
        os.remove(args.modelWeights)

    # Save the model
    json_string = model.to_json()
    with open(args.modelJSON,'w' ) as f:
        json.dump(json_string, f)
    model.save_weights(args.modelWeights)
    print('Model saved at %s, weights saved at %s' %(args.modelJSON, args.modelWeights))
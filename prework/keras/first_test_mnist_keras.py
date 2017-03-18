#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
from keras.datasets import mnist

from keras.models import Sequential
from keras.layers import Dense
from keras.utils import np_utils
import numpy
import matplotlib.pyplot as plt



(x_train,y_train),(x_test,y_test) = mnist.load_data()
#asd = plt.imshow(x_train[1],cmap='gray')

# Get the number of pixels
numPixels = x_train[0].shape[0] * x_train[0].shape[1]
numTrainImages = x_train.shape[0]
numTestImages = x_test.shape[0]

# Reshape the data to be a column vector
x_train = x_train.reshape(numTrainImages,numPixels)
x_test  = x_test.reshape(numTestImages,numPixels)

x_train = x_train/255
x_test = x_test/255



y_train = np_utils.to_categorical(y_train)
y_test = np_utils.to_categorical(y_test)
numClasses = y_train.shape[1]

#Create a basic model
def baseline_model():
    model = Sequential()
    model.add(Dense(output_dim=numPixels,
                    input_dim=numPixels,
                    init='normal',
                    activation='relu'))
    model.add(Dense(numClasses,
                    init='normal',
                    activation='softmax'))
    # Compile the model
    model.compile(optimizer='adam',
                  loss = 'categorical_crossentropy',
                  metrics=['accuracy'])
    return model
    
#Build the model
model = baseline_model()

#Fit the model
history = model.fit(x_train,y_train,batch_size=200,nb_epoch=10,
          validation_data=(x_test,y_test),verbose=2)

#Final evaluation of the model
scores  = model.evaluate(x_test,y_test,verbose=0)



    
    


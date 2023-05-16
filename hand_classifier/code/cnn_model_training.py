import os
import numpy as np
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import Adam

# Set the data directory
data_dir = '/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/cnn_data'

# Get the class names from the folder names
class_names = os.listdir(data_dir)

# Create empty lists to hold the data and labels
data = []
labels = []

# Loop over each class and collect the data
for i, class_name in enumerate(class_names):
    class_dir = os.path.join(data_dir, class_name)
    file_names = os.listdir(class_dir)
    for file_name in file_names:
        file_path = os.path.join(class_dir, file_name)
        # Load the array from the .npy file
        array = np.load(file_path)
        array =np.asarray(array)
        # Append the flattened array and label to the data lists
        data.append(array.flatten())
        labels.append(i)

# Convert the data and labels to NumPy arrays
data = np.array(data)
labels = np.array(labels)

# Split the data into training and testing sets

train_data, test_data, train_labels, test_labels = train_test_split(data, labels, test_size=0.2, random_state=42)
print(train_data.shape)
print(test_data.shape)
# Convert the NumPy arrays to TensorFlow tensors
train_data = tf.convert_to_tensor(train_data)
test_data = tf.convert_to_tensor(test_data)
train_labels = tf.convert_to_tensor(train_labels)
test_labels = tf.convert_to_tensor(test_labels)
# Define the model architecture
model = Sequential()
model.add(Dense(64, activation='relu', input_shape=(210,)))
model.add(Dropout(0.5))
model.add(Dense(64, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(len(class_names), activation='softmax'))

# Compile the model
model.compile(loss='sparse_categorical_crossentropy', optimizer=Adam(), metrics=['accuracy'])

# Train the model
model.fit(train_data, train_labels, batch_size=32, epochs=10, validation_data=(test_data, test_labels))

# Save the model
model.save('/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/keras_model.h5')

# Save the labels to a text file
with open('/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/labels.txt', 'w') as f:
    for label in class_names:
        f.write(label + '\n')

# Evaluate the model on the testing data
test_loss, test_acc = model.evaluate(test_data, test_labels)
print('Test loss:', test_loss)
print('Test accuracy:', test_acc)

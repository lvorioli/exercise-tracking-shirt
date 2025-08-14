# Luke Orioli
from sklearn.model_selection import train_test_split
import pandas as pd
from sklearn.metrics import precision_score, recall_score, f1_score, accuracy_score, ConfusionMatrixDisplay, confusion_matrix, roc_curve, auc
import matplotlib.pyplot as plt
import numpy as np
import joblib
from sklearn.preprocessing import LabelEncoder
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense, Dropout

file_name = "exercise_data_all.csv"

print(f"Reading data from {file_name}...")
df = pd.read_csv(file_name)

# Exercise data columns
sensor_columns = ['Tilt', 'Touch', 'Magnet', 'Acc X', 'Acc Y', 'Acc Z', 'Gyro X', 'Gyro Y', 'Gyro Z']

# Group by 'Movement' and 'Movement Count'
grouped = df.groupby(['Movement', 'Movement Count'])

# Determine the shortest sequence length
min_len = min(len(group) for _, group in grouped)

# Truncate each sequence to the shortest length
X = np.array([
    group[sensor_columns].values[:min_len]
    for _, group in grouped
], dtype=np.float32)

# Extract labels for each sequence
y = np.array([label for (label, _) in grouped.groups.keys()])

_reps, samples_per_movement, num_features = X.shape

# Transform the categorical label strings into integers using an encoder, 
# and save the encoder.
le = LabelEncoder()
y = le.fit_transform(y)
joblib.dump(le, 'label_encoder.pkl')

print("Splitting data...")

# Split the data into training (80 %) and testing (20 %) data.
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=11)

# Initialize a 1-D CNN model.
model = Sequential([
    Conv1D(64, kernel_size=5, activation='relu', input_shape=(samples_per_movement, num_features)),
    MaxPooling1D(pool_size=2),
    Conv1D(128, kernel_size=5, activation='relu'),
    MaxPooling1D(pool_size=2),
    Flatten(),
    Dense(128, activation='relu'),
    Dropout(0.5),
    Dense(len(np.unique(y)), activation='softmax')
])

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# Train the model.
history = model.fit(X_train, y_train, epochs=20, batch_size=32, validation_data=(X_test, y_test))

# Test the model.
loss, accuracy = model.evaluate(X_test, y_test)
print(f"Test Accuracy: {accuracy * 100:.2f}%")

y_pred_probs = model.predict(X_test)
y_pred = np.argmax(y_pred_probs, axis=1)

# Get the metrics of the model.
precision = precision_score(y_test, y_pred, average='weighted')
recall = recall_score(y_test, y_pred, average='weighted')
f1 = f1_score(y_test, y_pred, average='weighted')
accuracy = accuracy_score(y_test, y_pred)
metric_avg = (precision + recall + f1 + accuracy) / 4.0

print(f"Precision: {precision:.4f}")
print(f"Recall: {recall:.4f}")
print(f"F1: {f1:.4f}")
print(f"Accuracy: {accuracy:.4f}")

# Get all classes and move "Rest ()" to the end.
labels = list(le.classes_)
if "Rest ()" in labels:
    labels.remove("Rest ()")
    labels.append("Rest ()")

# Get indices of labels in new order.
label_indices = [np.where(le.classes_ == lbl)[0][0] for lbl in labels]

# Compute confusion matrix using reordered labels.
cm = confusion_matrix(y_test, y_pred, labels=label_indices)

# Plot the confusion matrix.
fig, ax = plt.subplots(figsize=(12, 10))
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=labels)
disp.plot(cmap=plt.cm.Blues, ax=ax, xticks_rotation=45)
plt.xticks(rotation=45, ha='right')
plt.title("1D CNN Model Confusion Matrix")
plt.tight_layout()
plt.show()

# Save the trained model.
model.save('model.keras')
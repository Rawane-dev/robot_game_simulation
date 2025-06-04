## Gazebo Models

This folder contains all the 3D models used in the Connect 4 robot game simulation.

### 📁 Where to Place the Models

To ensure Gazebo can find and use these models correctly, place these folders (babyblue, brown, green, mustard, orange, pink, purple, Board, cube_colored)in your local Gazebo models directory:

```bash
~/.gazebo/models/

### 📦 Included Models

#### 🟦 Game Board
- A 3D model of the Connect 4 board created in **Blender**.
- Serves as the main structure for the game.
- This model is **static** and should be placed at the center of the play area on a table in Gazebo if a new world is created .

#### 🟨🟥 Graspable Cubes (Red and Yellow)
- Two separate cube models representing the **red** and **yellow** game pieces.
- The **Yellow** cube is designed to be picked up, moved, and placed by the robot.
- The **Red** cube is used for gameplay during the human's turn.

#### 📦 Static Cubes
- Additional colored cubes meant to float above the board.
- These are **static models**, used for detecting the exact position of the column.
- Not meant to be grasped or moved.

```

## Visual SLAM

Based on estimating relative camera poses from a sequence of images.

### Dataset

[The KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
dataset will be used for this implementation. It has a total of 22 stereo sequences, 11
of which also have ground-truth poses in order to validate accuracy.

We will only make use of the stereo sequences (there are no odometry measurements).

### Back End
- Create similar Graph class, however we will now be using SE3 objects instead of SE2. 
- Again, Graph Optimization will be done via the g2o module.

### Front End

- Parsing the dataset. Use this nice tool https://github.com/utiasSTARS/pykitti

- Updating and constructing the graph:

- **For** all frames in the dataset:
  - **Compute** relative pose of successive frame:
    - Use SIFT to find features in both frames and matches
    - Use RANSAC to find Essential Matrix E
    - Recover the pose (use opencv.recoverPose())
  
  - **If** large enough change is detected:
    - Add vertices corresponding to the two different frames to the graph
    - Add the edge connecting them (relative pose)
    - **Do** loop closure detection:
      - Put all vertices of Graph in KD-Tree
      - Query only close poses according to KD-Tree
        - **For each** 'close' pose:
          - **Compute** relative pose between close pose and current pose
          - **If** relative pose (transformation) is good enough:
            - A loop closure is detected
            - Add edge to graph using computed relative pose
            - Add vertex to graph
            - Optimize graph
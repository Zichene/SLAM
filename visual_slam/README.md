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

- **For** all frame in the dataset:
  - 
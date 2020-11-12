name: charlie denhart
url:  https://www.youtube.com/watch?v=gB-3ltQIvKA
bug fix pr: https://github.com/NatTuck/cs5335hw-gazebo/pull/8
comments: 
my strategy was roughly a right wall following strategy, except the robot went straight after clearing a wall instead of tending back left.  My visualization strategy was similar to that shown in the provided demonstration videos; I used bayes rule to update belief about cell states.  I cleared the canvas and rerendered points periodically to allow cells that had been updated negatively to be unrendered.

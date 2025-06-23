# ECE 276B Project 1 • Dynamic Programming for Autonomous Navigation

![Demo](./media/example.gif)

> **Code is private.** Contact me for access.

## One‑sentence overview

Dynamic programming drives an agent through door‑and‑key mazes to the goal in the fewest moves, producing policies that generalize across 40 test worlds.

## Applications

* **Planning under uncertainty:** Complete MDP formulation and optimal control.
* **Efficient policy reuse:** One policy handles thirty‑six unseen maps without retraining.
* **Clean robotics stack:** Numpy‑vectorized DP, fast visualization, and automated GIF reporting.

## Quick technical summary

| Item        | Details                                                     |
| ----------- | ----------------------------------------------------------- |
| Environment | Grid world with walls, keyed doors, moving agent            |
| State       | $(x, y, v, k, d\_1, d\_2, p\_k, p\_g)$ up to 18 431 states  |
| Actions     | Move Forward, Turn Left, Turn Right, Pick Key, Unlock Door  |
| Cost        | Unit per move, infinity for collisions                      |
| Solver      | Finite horizon value iteration with early convergence check |
| Performance | Full benchmark suite runs in < 8 s on laptop                |

---

# Results

After using my implementation of the DP algorithm to find the optimal policy for each of the 7 known environments and the 36 random maps, the agent's performance was evaluated in each of the environments.

## Part A: Known Map

### 5x5 Normal

Sequence of actions for the 5x5 Normal environment:

\['TL', 'TL', 'PK', 'TR', 'UD', 'MF', 'MF', 'TR', 'MF']

Cost: 9

|                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_A/1/0.png) | ![](starter_code/images/part_A/1/1.png) | ![](starter_code/images/part_A/1/2.png) | ![](starter_code/images/part_A/1/3.png) |
| ![](starter_code/images/part_A/1/4.png) | ![](starter_code/images/part_A/1/5.png) | ![](starter_code/images/part_A/1/6.png) | ![](starter_code/images/part_A/1/7.png) |
| ![](starter_code/images/part_A/1/8.png) | ![](starter_code/images/part_A/1/9.png) |                                         |                                         |

### 6x6 Direct

Sequence of actions for the 6x6 Direct environment:

\['MF', 'MF', 'TR', 'MF', 'MF']

Cost: 5

|                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_A/2/0.png) | ![](starter_code/images/part_A/2/1.png) | ![](starter_code/images/part_A/2/2.png) |
| ![](starter_code/images/part_A/2/3.png) | ![](starter_code/images/part_A/2/4.png) | ![](starter_code/images/part_A/2/5.png) |

### 6x6 Normal

Sequence of actions for the 6x6 Normal environment:

\['TL', 'MF', 'PK', 'TL', 'MF', 'TL', 'MF', 'TR', 'UD', 'MF', 'MF', 'TR', 'MF']

Cost: 13

|                                          |                                          |                                          |                                          |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_A/3/0.png)  | ![](starter_code/images/part_A/3/1.png)  | ![](starter_code/images/part_A/3/2.png)  | ![](starter_code/images/part_A/3/3.png)  |
| ![](starter_code/images/part_A/3/4.png)  | ![](starter_code/images/part_A/3/5.png)  | ![](starter_code/images/part_A/3/6.png)  | ![](starter_code/images/part_A/3/7.png)  |
| ![](starter_code/images/part_A/3/8.png)  | ![](starter_code/images/part_A/3/9.png)  | ![](starter_code/images/part_A/3/10.png) | ![](starter_code/images/part_A/3/11.png) |
| ![](starter_code/images/part_A/3/12.png) | ![](starter_code/images/part_A/3/13.png) |                                          |                                          |

### 6x6 Shortcut

Sequence of actions for the 6x6 Shortcut environment:

\['PK', 'TL', 'TL', 'UD', 'MF', 'MF']

Cost: 6

|                                         |                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_A/4/0.png) | ![](starter_code/images/part_A/4/1.png) | ![](starter_code/images/part_A/4/2.png) | ![](starter_code/images/part_A/4/3.png) | ![](starter_code/images/part_A/4/4.png) |
| ![](starter_code/images/part_A/4/5.png) | ![](starter_code/images/part_A/4/6.png) |                                         |                                         |                                         |

### 8x8 Direct

Sequence of actions for the 8x8 Direct environment:

\['MF', 'TL', 'MF', 'MF', 'MF', 'TL', 'MF']

Cost: 7

|                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_A/5/0.png) | ![](starter_code/images/part_A/5/1.png) | ![](starter_code/images/part_A/5/2.png) | ![](starter_code/images/part_A/5/3.png) |
| ![](starter_code/images/part_A/5/4.png) | ![](starter_code/images/part_A/5/5.png) | ![](starter_code/images/part_A/5/6.png) | ![](starter_code/images/part_A/5/7.png) |

### 8x8 Normal

Sequence of actions for the 8x8 Normal environment:

\['TR', 'MF', 'TL', 'MF', 'TR', 'MF', 'MF', 'MF', 'PK', 'TL', 'TL', 'MF', 'MF', 'MF', 'TR', 'UD', 'MF', 'MF', 'MF', 'TR', 'MF', 'MF', 'MF']

Cost: 23

|                                          |                                          |                                          |                                          |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_A/6/0.png)  | ![](starter_code/images/part_A/6/1.png)  | ![](starter_code/images/part_A/6/2.png)  | ![](starter_code/images/part_A/6/3.png)  |
| ![](starter_code/images/part_A/6/4.png)  | ![](starter_code/images/part_A/6/5.png)  | ![](starter_code/images/part_A/6/6.png)  | ![](starter_code/images/part_A/6/7.png)  |
| ![](starter_code/images/part_A/6/8.png)  | ![](starter_code/images/part_A/6/9.png)  | ![](starter_code/images/part_A/6/10.png) | ![](starter_code/images/part_A/6/11.png) |
| ![](starter_code/images/part_A/6/12.png) | ![](starter_code/images/part_A/6/13.png) | ![](starter_code/images/part_A/6/14.png) | ![](starter_code/images/part_A/6/15.png) |
| ![](starter_code/images/part_A/6/16.png) | ![](starter_code/images/part_A/6/17.png) | ![](starter_code/images/part_A/6/18.png) | ![](starter_code/images/part_A/6/19.png) |
| ![](starter_code/images/part_A/6/20.png) | ![](starter_code/images/part_A/6/21.png) | ![](starter_code/images/part_A/6/22.png) | ![](starter_code/images/part_A/6/23.png) |

### 8x8 Shortcut

Sequence of actions for the 8x8 Shortcut environment:

\['TR', 'MF', 'TR', 'PK', 'TL', 'UD', 'MF', 'MF']

Cost: 8

|                                         |                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_A/7/0.png) | ![](starter_code/images/part_A/7/1.png) | ![](starter_code/images/part_A/7/2.png) | ![](starter_code/images/part_A/7/3.png) | ![](starter_code/images/part_A/7/4.png) |
| ![](starter_code/images/part_A/7/5.png) | ![](starter_code/images/part_A/7/6.png) | ![](starter_code/images/part_A/7/7.png) | ![](starter_code/images/part_A/7/8.png) |                                         |

### 8x8 Example

Sequence of actions for the 8x8 Example environment:

\['TR', 'MF', 'PK', 'TL', 'UD', 'MF', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 11

|                                         |                                         |                                          |                                          |
| --------------------------------------- | --------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_A/8/0.png) | ![](starter_code/images/part_A/8/1.png) | ![](starter_code/images/part_A/8/2.png)  | ![](starter_code/images/part_A/8/3.png)  |
| ![](starter_code/images/part_A/8/4.png) | ![](starter_code/images/part_A/8/5.png) | ![](starter_code/images/part_A/8/6.png)  | ![](starter_code/images/part_A/8/7.png)  |
| ![](starter_code/images/part_A/8/8.png) | ![](starter_code/images/part_A/8/9.png) | ![](starter_code/images/part_A/8/10.png) | ![](starter_code/images/part_A/8/11.png) |

## Part B: Random Maps

The optimal policy obtained from DP is used to navigate the agent in the 36 random maps. The agent's performance is evaluated in each of the random maps.

**Note:** Since there are several random maps, only a few random maps are shown here as a sample. All the resulting .gif files are saved in the `gif` folder in the **Code** submission on Gradescope.

### Random Map 1

Action Sequence: \['TR', 'MF', 'MF', 'TR', 'MF']

Cost: 5

|                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_B/1/0.png) | ![](starter_code/images/part_B/1/1.png) | ![](starter_code/images/part_B/1/2.png) | ![](starter_code/images/part_B/1/3.png) |
| ![](starter_code/images/part_B/1/4.png) | ![](starter_code/images/part_B/1/5.png) |                                         |                                         |

### Random Map 2

\['MF', 'MF', 'MF', 'MF', 'TL', 'MF', 'PK', 'TL', 'MF', 'TL', 'MF', 'UD', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 17

|                                          |                                          |                                          |                                          |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_B/2/0.png)  | ![](starter_code/images/part_B/2/1.png)  | ![](starter_code/images/part_B/2/2.png)  | ![](starter_code/images/part_B/2/3.png)  |
| ![](starter_code/images/part_B/2/4.png)  | ![](starter_code/images/part_B/2/5.png)  | ![](starter_code/images/part_B/2/6.png)  | ![](starter_code/images/part_B/2/7.png)  |
| ![](starter_code/images/part_B/2/8.png)  | ![](starter_code/images/part_B/2/9.png)  | ![](starter_code/images/part_B/2/10.png) | ![](starter_code/images/part_B/2/11.png) |
| ![](starter_code/images/part_B/2/12.png) | ![](starter_code/images/part_B/2/13.png) | ![](starter_code/images/part_B/2/14.png) | ![](starter_code/images/part_B/2/15.png) |
| ![](starter_code/images/part_B/2/16.png) | ![](starter_code/images/part_B/2/17.png) |                                          |                                          |

### Random Map 3

\['TR', 'MF', 'MF', 'TR', 'MF']

Cost: 5

|                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_B/3/0.png) | ![](starter_code/images/part_B/3/1.png) | ![](starter_code/images/part_B/3/2.png) | ![](starter_code/images/part_B/3/3.png) |
| ![](starter_code/images/part_B/3/4.png) | ![](starter_code/images/part_B/3/5.png) |                                         |                                         |

### Random Map 4

\['TR', 'MF', 'MF', 'TR', 'MF']

Cost: 5

|                                         |                                         |                                         |                                         |
| --------------------------------------- | --------------------------------------- | --------------------------------------- | --------------------------------------- |
| ![](starter_code/images/part_B/4/0.png) | ![](starter_code/images/part_B/4/1.png) | ![](starter_code/images/part_B/4/2.png) | ![](starter_code/images/part_B/4/3.png) |
| ![](starter_code/images/part_B/4/4.png) | ![](starter_code/images/part_B/4/5.png) |                                         |                                         |

### Random Map 5

\['MF', 'MF', 'TL', 'PK', 'TR', 'MF', 'TR', 'UD', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 13

|                                          |                                          |                                          |                                          |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_B/5/0.png)  | ![](starter_code/images/part_B/5/1.png)  | ![](starter_code/images/part_B/5/2.png)  | ![](starter_code/images/part_B/5/3.png)  |
| ![](starter_code/images/part_B/5/4.png)  | ![](starter_code/images/part_B/5/5.png)  | ![](starter_code/images/part_B/5/6.png)  | ![](starter_code/images/part_B/5/7.png)  |
| ![](starter_code/images/part_B/5/8.png)  | ![](starter_code/images/part_B/5/9.png)  | ![](starter_code/images/part_B/5/10.png) | ![](starter_code/images/part_B/5/11.png) |
| ![](starter_code/images/part_B/5/12.png) | ![](starter_code/images/part_B/5/13.png) |                                          |                                          |

### Random Map 6

\['MF', 'MF', 'MF', 'TR', 'MF', 'MF', 'TR', 'MF', 'MF', 'MF', 'MF']

Cost: 11

|                                         |                                         |                                          |                                          |
| --------------------------------------- | --------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_B/6/0.png) | ![](starter_code/images/part_B/6/1.png) | ![](starter_code/images/part_B/6/2.png)  | ![](starter_code/images/part_B/6/3.png)  |
| ![](starter_code/images/part_B/6/4.png) | ![](starter_code/images/part_B/6/5.png) | ![](starter_code/images/part_B/6/6.png)  | ![](starter_code/images/part_B/6/7.png)  |
| ![](starter_code/images/part_B/6/8.png) | ![](starter_code/images/part_B/6/9.png) | ![](starter_code/images/part_B/6/10.png) | ![](starter_code/images/part_B/6/11.png) |

### Random Map 7

\['MF', 'MF', 'MF', 'TR', 'MF', 'MF', 'TR', 'MF', 'MF', 'MF', 'MF']

Cost: 11

|                                         |                                         |                                          |                                          |
| --------------------------------------- | --------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_B/7/0.png) | ![](starter_code/images/part_B/7/1.png) | ![](starter_code/images/part_B/7/2.png)  | ![](starter_code/images/part_B/7/3.png)  |
| ![](starter_code/images/part_B/7/4.png) | ![](starter_code/images/part_B/7/5.png) | ![](starter_code/images/part_B/7/6.png)  | ![](starter_code/images/part_B/7/7.png)  |
| ![](starter_code/images/part_B/7/8.png) | ![](starter_code/images/part_B/7/9.png) | ![](starter_code/images/part_B/7/10.png) | ![](starter_code/images/part_B/7/11.png) |

### Random Map 30

\['TR', 'MF', 'MF', 'MF', 'TL', 'MF', 'MF']

Cost: 7

|                                          |                                          |                                          |                                          |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_B/30/0.png) | ![](starter_code/images/part_B/30/1.png) | ![](starter_code/images/part_B/30/2.png) | ![](starter_code/images/part_B/30/3.png) |
| ![](starter_code/images/part_B/30/4.png) | ![](starter_code/images/part_B/30/5.png) | ![](starter_code/images/part_B/30/6.png) | ![](starter_code/images/part_B/30/7.png) |

### Random Map 36

\['MF', 'MF', 'MF', 'TR', 'MF', 'MF', 'TL', 'MF']

Cost: 8

|                                          |                                          |                                          |                                          |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| ![](starter_code/images/part_B/36/0.png) | ![](starter_code/images/part_B/36/1.png) | ![](starter_code/images/part_B/36/2.png) | ![](starter_code/images/part_B/36/3.png) |
| ![](starter_code/images/part_B/36/4.png) | ![](starter_code/images/part_B/36/5.png) | ![](starter_code/images/part_B/36/6.png) | ![](starter_code/images/part_B/36/7.png) |
| ![](starter_code/images/part_B/36/8.png) |                                          |                                          |                                          |

## Agent with Different Starting Positions and Directions

To test part B even further, it is good to investigate the effect of different starting positions and directions for the agent.

### Starting Position: (2,3), Direction: Left

After changing the starting position and direction, the agent still successfully navigates to the goal using the optimal policy obtained from DP.

\['TL', 'MF', 'MF', 'TL', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 9

|                                           |                                           |                                           |                                           |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| ![](starter_code/images/part_B_1/1/0.png) | ![](starter_code/images/part_B_1/1/1.png) | ![](starter_code/images/part_B_1/1/2.png) | ![](starter_code/images/part_B_1/1/3.png) |
| ![](starter_code/images/part_B_1/1/4.png) | ![](starter_code/images/part_B_1/1/5.png) | ![](starter_code/images/part_B_1/1/6.png) | ![](starter_code/images/part_B_1/1/7.png) |
| ![](starter_code/images/part_B_1/1/8.png) | ![](starter_code/images/part_B_1/1/9.png) |                                           |                                           |

Another example:

Optimal Policy: \['MF', 'TR', 'MF', 'PK', 'TR', 'MF', 'MF', 'UD', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 13

|                                            |                                            |                                            |                                            |
| ------------------------------------------ | ------------------------------------------ | ------------------------------------------ | ------------------------------------------ |
| ![](starter_code/images/part_B_1/2/0.png)  | ![](starter_code/images/part_B_1/2/1.png)  | ![](starter_code/images/part_B_1/2/2.png)  | ![](starter_code/images/part_B_1/2/3.png)  |
| ![](starter_code/images/part_B_1/2/4.png)  | ![](starter_code/images/part_B_1/2/5.png)  | ![](starter_code/images/part_B_1/2/6.png)  | ![](starter_code/images/part_B_1/2/7.png)  |
| ![](starter_code/images/part_B_1/2/8.png)  | ![](starter_code/images/part_B_1/2/9.png)  | ![](starter_code/images/part_B_1/2/10.png) | ![](starter_code/images/part_B_1/2/11.png) |
| ![](starter_code/images/part_B_1/2/12.png) | ![](starter_code/images/part_B_1/2/13.png) |                                            |                                            |

Another example:

\['TR', 'MF', 'TR', 'MF', 'MF', 'MF', 'TR', 'MF', 'MF', 'MF', 'MF']

Cost: 11

|                                           |                                           |                                            |                                            |
| ----------------------------------------- | ----------------------------------------- | ------------------------------------------ | ------------------------------------------ |
| ![](starter_code/images/part_B_1/6/0.png) | ![](starter_code/images/part_B_1/6/1.png) | ![](starter_code/images/part_B_1/6/2.png)  | ![](starter_code/images/part_B_1/6/3.png)  |
| ![](starter_code/images/part_B_1/6/4.png) | ![](starter_code/images/part_B_1/6/5.png) | ![](starter_code/images/part_B_1/6/6.png)  | ![](starter_code/images/part_B_1/6/7.png)  |
| ![](starter_code/images/part_B_1/6/8.png) | ![](starter_code/images/part_B_1/6/9.png) | ![](starter_code/images/part_B_1/6/10.png) | ![](starter_code/images/part_B_1/6/11.png) |

### Starting Position: (0,4), Direction: Down

After changing the starting position and direction again, the agent still successfully navigates to the goal using the optimal policy obtained from DP.

One example:

Optimal Policy: \['MF', 'TL', 'MF', 'MF', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 9

|                                           |                                           |                                           |                                           |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| ![](starter_code/images/part_B_2/1/0.png) | ![](starter_code/images/part_B_2/1/1.png) | ![](starter_code/images/part_B_2/1/2.png) | ![](starter_code/images/part_B_2/1/3.png) |
| ![](starter_code/images/part_B_2/1/4.png) | ![](starter_code/images/part_B_2/1/5.png) | ![](starter_code/images/part_B_2/1/6.png) | ![](starter_code/images/part_B_2/1/7.png) |
| ![](starter_code/images/part_B_2/1/8.png) | ![](starter_code/images/part_B_2/1/9.png) |                                           |                                           |

Another example:

Optimal Policy: \['TL', 'MF', 'TL', 'MF', 'MF', 'PK', 'TR', 'MF', 'MF', 'UD', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 15

|                                            |                                            |                                            |                                            |
| ------------------------------------------ | ------------------------------------------ | ------------------------------------------ | ------------------------------------------ |
| ![](starter_code/images/part_B_2/2/0.png)  | ![](starter_code/images/part_B_2/2/1.png)  | ![](starter_code/images/part_B_2/2/2.png)  | ![](starter_code/images/part_B_2/2/3.png)  |
| ![](starter_code/images/part_B_2/2/4.png)  | ![](starter_code/images/part_B_2/2/5.png)  | ![](starter_code/images/part_B_2/2/6.png)  | ![](starter_code/images/part_B_2/2/7.png)  |
| ![](starter_code/images/part_B_2/2/8.png)  | ![](starter_code/images/part_B_2/2/9.png)  | ![](starter_code/images/part_B_2/2/10.png) | ![](starter_code/images/part_B_2/2/11.png) |
| ![](starter_code/images/part_B_2/2/12.png) | ![](starter_code/images/part_B_2/2/13.png) | ![](starter_code/images/part_B_2/2/14.png) | ![](starter_code/images/part_B_2/2/15.png) |

Another example:

Optimal Policy: \['MF', 'TL', 'MF', 'MF', 'MF', 'MF', 'MF', 'TR', 'MF']

Cost: 9

|                                           |                                           |                                           |                                           |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| ![](starter_code/images/part_B_2/3/0.png) | ![](starter_code/images/part_B_2/3/1.png) | ![](starter_code/images/part_B_2/3/2.png) | ![](starter_code/images/part_B_2/3/3.png) |
| ![](starter_code/images/part_B_2/3/4.png) | ![](starter_code/images/part_B_2/3/5.png) | ![](starter_code/images/part_B_2/3/6.png) | ![](starter_code/images/part_B_2/3/7.png) |
| ![](starter_code/images/part_B_2/3/8.png) | ![](starter_code/images/part_B_2/3/9.png) |                                           |                                           |

### Starting Position: (6,7), Direction: Right

One example:

\['TL', 'MF', 'TL', 'MF']

Cost: 4

|                                           |                                           |                                           |                                           |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| ![](starter_code/images/part_B_3/1/0.png) | ![](starter_code/images/part_B_3/1/1.png) | ![](starter_code/images/part_B_3/1/2.png) | ![](starter_code/images/part_B_3/1/3.png) |
| ![](starter_code/images/part_B_3/1/4.png) |                                           |                                           |                                           |

Another example:

\['TL', 'MF', 'MF', 'MF', 'MF']

Cost: 5

|                                           |                                           |                                           |                                           |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| ![](starter_code/images/part_B_3/2/0.png) | ![](starter_code/images/part_B_3/2/1.png) | ![](starter_code/images/part_B_3/2/2.png) | ![](starter_code/images/part_B_3/2/3.png) |
| ![](starter_code/images/part_B_3/2/4.png) | ![](starter_code/images/part_B_3/2/5.png) |                                           |                                           |

Another example:

\['TL', 'MF', 'TL', 'MF']

Cost: 4

|                                           |                                           |                                           |                                           |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| ![](starter_code/images/part_B_3/3/0.png) | ![](starter_code/images/part_B_3/3/1.png) | ![](starter_code/images/part_B_3/3/2.png) | ![](starter_code/images/part_B_3/3/3.png) |
| ![](starter_code/images/part_B_3/3/4.png) |                                           |                                           |                                           |

## Discussion of Performance

Part A: The policy is optimal in all seven known maps.  Vectorized DP finishes quickly.

Part B: A single policy solves thirty‑six unknown maps and variations in start state.  This shows strong generalization and low runtime cost.
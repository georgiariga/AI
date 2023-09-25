# File:         a_star_DM.r 
# Description:  A* algorithm implementation for Delivery Man
# Author:       Derrick Nii Adjei Adjei


# Install the package
# install.packages("DeliveryMan_1.1.0.tar.gz", repos = NULL, type="source")

# Load the library
library("DeliveryMan")

# Read documentation
# ?runDeliveryMan
# ?testDM

# A priority queue which allows to insert elements and order them by priority
# Source: http://rosettacode.org/wiki/Priority_queue#R
PriorityQueue <- function(){
  queueKeys <<- queueValues <<- NULL
  insert <- function(key, value){
    # If node already exists on queue, and this new addition is better,
    # delete previous one and insert this new one instead
    index = getValueIndex(value)
    if(length(index) > 0){
      if(isTRUE(key < queueKeys[[index]])){
        queueKeys <<- queueKeys[-index]
        queueValues <<- queueValues[-index]
      }else{
        # Ignore it, we already have a cheaper path
        return(-1)
      }
    }
    
    # Insert new value in queue
    temp <- c(queueKeys, key)
    ord <- order(temp)
    queueKeys <<- temp[ord]
    queueValues <<- c(queueValues, list(value))[ord]
  }
  
  pop <- function(){
    head <- queueValues[[1]]
    queueValues <<- queueValues[-1]
    queueKeys <<- queueKeys[-1]
    return (head)
  }
  
  empty <- function() length(queueKeys) == 0
  getValueIndex <- function(value) which(queueValues %in% list(value) == TRUE)
  list(insert = insert, pop = pop, empty = empty)
}

# A list which allows insertion of elements to it and verify if a particular element exists or not
List <- function(){
  listValues <<- NULL
  insert <- function(value) listValues <<- c(listValues, list(value))
  exists <- function(value) isTRUE(which(listValues %in% list(value) == TRUE) > 0)
  getValues <- function() listValues
  
  list(insert = insert, exists = exists, getValues = getValues)
}

# function to compute manhattan distance between two nodes
getManhattanDistance <- function(from, to){
  return (abs(from[1] - to[1]) + abs(from[2] - to[2]))
}

# function to get Euclidean distance between two locations
getEuclideanDistance <- function(from, to){
  return (sqrt((from[1]-to[1])**2 + (from[2] - to[2])**2))
}

# function to find a node index given node coordinates
getNodeIndex <- function(nodeList, neighbor){
  if(length(nodeList) == 0){
    return(0)
  }
  
  index = which(sapply(nodeList, function(node)(node$x == neighbor$x && node$y==neighbor$y)))
  ifelse(length(index) == 0, return(0), return(index))
}

# Calculate the cost of a vertical edge
getVerticalEdgeCost <- function(roads, from, to){
  if(from[2] < to[2]){
    # going up
    return (roads$vroads[from[2], from[1]])
  }else{
    # moving down
    return (roads$vroads[to[2], to[1]])
  }
}

# Calculate cost of a horizontal edge
getHorizontalEdgeCost <- function(roads, from, to){
  if(from[1] > to[1]){
    # going left
    return (roads$hroads[to[2], to[1]])
  }else{
    # going right
    return (roads$hroads[from[2], from[1]])
  }
}

# Calculate edge cost from current position to neighbor position
getEdgeCost <- function(roads, path){
  cost <- 0
  for(i in 1:(length(path) -1)){
    isMovingVertically <- path[[i]][1] == path[[i+1]][1]
    if(isMovingVertically){
      cost <- cost + getVerticalEdgeCost(roads, path[[i]], path[[i+1]])
    }else{
      cost <- cost + getHorizontalEdgeCost(roads, path[[i]], path[[i+1]])
    }
  }
  
  return (cost)
}

# Calculate the cost of an edge and a heuristic
getTotalCost <- function(roads, path, goal){
  to = path[[length(path)]][1:2]
  return (getEdgeCost(roads, path) + getManhattanDistance(to, goal))
}

# Get all available neighbors given a node
getNeighbors <- function(x, y, xSize, ySize){
  neighbors = matrix(nrow=4, ncol=2, byrow = TRUE)
  
  # add all possible horizontal and vertical neighbors
  neighbors[,1] = c(x-1, x, x, x+1)
  neighbors[,2] = c(y, y+1, y-1, y)
  
  # ensure only valid neighbors added, remove all lower bound positions (< 0)
  neighbors = neighbors[neighbors[,1] > 0,]
  neighbors = neighbors[neighbors[,2] > 0,]
  
  # remove all upper bound positions (< size of matrix)
  neighbors = neighbors[neighbors[,1] < xSize+1,]
  neighbors = neighbors[neighbors[,2] < ySize+1,]
  
  return (neighbors)
}

# Check if node is goal
isGoal <- function(neighbor, goal){
  return (goal[1] == neighbor[1] && goal[2] == neighbor[2])
}

# Convert a vector representation of node to string
nodeToString <- function(node){
  return (paste(node[1], node[2], sep = ','))
}

# Convert string representation of a node to a vector
stringToNode <- function(nodeString){
  strNode = strsplit(nodeString, ',')[[1]]
  x = as.integer(strNode[1])
  y = as.integer(strNode[2])
  
  return (c(x,y))
}

# Get path from an initial position to the goal position given the path visited by the algorithm
generatePath = function(from, to, path){
  goal = nodeToString(from)
  currentNode = nodeToString(to)
  
  # create path visited by traversing the path variable
  # from goal to initial position (in reverse order)
  vectors <- list(c(to))
  
  while(currentNode != goal){
    node = stringToNode(path[[currentNode]])
    vectors = c(vectors, list(node))
    currentNode = path[[currentNode]]
  }
  print(path)
  # Return path from initial position to goal
  return (rev(vectors))
}

# Perform A* search from current car location towards goal
# Algorithm implemented based on slides
aStarSearch <- function(from, to, roads, packages){
  # Get the matrix size
  xSize = dim(roads$hroads)[1]
  ySize = dim(roads$vroads)[2]
  
  # Initialize visited, frontier, and path lists
  visited = List()
  frontier = PriorityQueue()
  path = list()
  
  # Put the starting location on the frontier (with cost 0)
  frontier$insert(0, from)
  
  while(!frontier$empty()){
    # Get node with the least f on the frontier
    node = frontier$pop()
    
    # Return the visited path + current node as path to goal
    if(isGoal(node, to)){
      return (generatePath(from, node, path))
    }
    
    neighbors = getNeighbors(node[1], node[2], xSize, ySize)
    
    for(i in 1:dim(neighbors)[1]){
      neighbor = neighbors[i,]
      # Only search neighbors which have not been already visited
      if(visited$exists(neighbor)){
        next
      }else{
        # Temporarily save visited path towards this neighbor
        tempPath = path
        tempPath[nodeToString(neighbor)] = nodeToString(node)
        
        # Attempt to add neighbor to frontier
        totalCost = getTotalCost(roads, generatePath(from, neighbor, tempPath), to)
        inserted = frontier$insert(totalCost, neighbor)
        
        # Add neighbor to path only if it was inserted in the frontier
        wasInserted = length(inserted) != 1 || inserted[[1]][1] != -1
        
        if (isTRUE(wasInserted)){
          path[nodeToString(neighbor)] = nodeToString(node)
        }
      }
    }
    
    # keep track of best path
    visited$insert(node)
  }
}

# Given a path, return the best next move car can make towards goal
generateNextMove = function(path){
  if(isTRUE(length(path) == 1)){
    # This happens when the package pickup and delivery locations are equal
    return(5)
  }
  
  currX = path[[1]][1]
  currY = path[[1]][2]
  nextX = path[[2]][1]
  nextY = path[[2]][2]
  
  # Move is horizontal
  if(isTRUE(nextX > currX)){
    return (6) # Right
  }
  
  if(isTRUE(nextX < currX)){
    return(4) # Left
  }
  
  # Move is vertical
  if(isTRUE(nextY > currY)){
    return(8) # UP
  }
  
  if(isTRUE(nextY < currY)){
    return(2) # Down
  }
  
  print('Error! Unable to find a suitable move.')
}

# Return a package pickup location which will be used as the goal for a particular search
getBestPackage <- function(from, packages){
  # Select closest package from current car's location as a pickup goal
  costs = NULL
  unpicked = packages[which(packages[, 5] %in% c(0) == TRUE),]
  
  if(isTRUE(length(unpicked) == 5)){
    # There's only 1 unpicked
    return (unpicked)
  }
  else {
    # Compute a weighted package + delivery location distance and choose the least value
    pickupWeight = 1
    deliveryWeight = 0
    
    for(i in 1:dim(unpicked)[1]){
      package = unpicked[i,]
      pickupLoc = package[1:2]
      deliveryLoc = package[3:4]
      pickupCost = getManhattanDistance(from, pickupLoc)
      deliveryCost = getEuclideanDistance(pickupLoc, deliveryLoc)
      costs = c(costs, (pickupCost * pickupWeight) + (deliveryCost * deliveryWeight))
    }
    
    return (unpicked[which.min(costs),])
  }
}

# Check if car is loaded
isLoaded = function(car) {
  return (car$load != 0)
}

# Get the delivery location of the package which is currently loaded
getDeliveryLocation <- function(packages){
  return (packages[which(packages[,5] %in% c(1) == TRUE),])
}

# Apply the A* Algorithm to DeliveryMan
aStarSearchDM <- function(roads, car, packages){
  from = c(car$x, car$y)
  to = NULL
  
  if(isLoaded(car)){
    to = getDeliveryLocation(packages)[3:4]
  }else{
    to = getBestPackage(from, packages)[1:2]
  }
  
  path = aStarSearch(from, to, roads, packages)
  car$nextMove <- generateNextMove(path)
  return (car)
}

runDeliveryMan(carReady = aStarSearchDM)
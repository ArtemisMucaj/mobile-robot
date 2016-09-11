NAME=mobile-robot
CONTAINER=ros-mobile-robot

# Build docker container
IMAGE=$(docker images | grep $CONTAINER)
if [[ $IMAGE == *"$CONTAINER"* ]];then
  echo "$CONTAINER already exists ..."
else
    docker build -t $CONTAINER .
fi

# Check if NAME is already running or exists
IS_RUNNING=$(docker ps | grep $NAME)
EXISTS=$(docker ps -a | grep $NAME)

if [[ $IS_RUNNING == *"$NAME"* ]];then
	echo "$NAME is already running"
else
	if [[ $EXISTS == *"$NAME"* ]];then
		echo "$NAME exists ... let's run it !"
		docker start $NAME
	else
		# Creating and provisionning a new container
		echo "$NAME app doesn't exist, wait a moment..."
		docker run -it -v $PWD:/home --name $NAME $CONTAINER
	fi
fi

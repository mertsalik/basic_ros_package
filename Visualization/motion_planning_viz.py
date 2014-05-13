import sys
import turtle

hor_block_widht = 5*20
hor_block_height = 1*20
ver_block_widht = 1*20
ver_block_height = 5*20
# tupple def -> hor x y
map1_obstacles = [(1,4,2),(1,0,-5),(0,-3,0),(0,-7,5)]
map2_obstacles = [(0,-3,0),(0,3,0),(1,-3,-7),(1,3,7),(0,6,-6),(0,-6,6)]
map3_obstacles = [(1,-5,6),(1,5,6),(0,-2,3),(0,2,3),(1,-5,0),(1,5,0),(0,-5,-3),(1,0.5,-4),(0,3.5,-6.5)]

def readGraph(t,file_name):
	f = open(file_name, 'r')
	node_count = 0
	first_line = f.readline()
	node_count = int(first_line)
	for i in range(node_count):
		actual_point = f.readline()
		adj_count = f.readline()
		adj_count = int(adj_count)
		adjs = []
		
		for j in range(adj_count):
			point = f.readline()
			adjs.append(point)
		point = actual_point.split(" ")
		
		for j in range(len(adjs)):
			temp = adjs[j]
			temp = temp.split(" ")
			p1_x = float(point[0])
			p1_y = float(point[1])
			p2_x = float(temp[0])
			p2_y = float(temp[1])
			drawLine(t, p1_x, p1_y, p2_x, p2_y, 'blue', 1)
		dummy = f.readline()
	print("finish")
    

def readPath(t, file_name):
	f = open(file_name, 'r')
	node_count = f.readline()
	node_count = int(node_count)
	points = []
	for i in range(node_count):
		temp = f.readline()
		points.append(temp)
	for i in range(len(points)-1):
		point = points[i]
		point = point.split(" ")
		p1_x = float(point[0])
		p1_y = float(point[1])
		point2 = points[i+1]
		point2 = point2.split(" ")
		p2_x = float(point2[0])
		p2_y = float(point2[1])
		drawLine(t, p1_x, p1_y, p2_x, p2_y, 'red', 2)

def border(t, screen_x, screen_y):
    """(Turtle, int, int)

    Draws a border around the canvas in red.
    """
    # Lift the pen and move the turtle to the center.
    t.penup()
    t.home()

    # Move to lower left corner of the screen; leaves the turtle
    # facing west.
    t.forward(screen_x / 2)
    t.right(90)
    t.forward(screen_y / 2)
    t.setheading(180)           # t.right(90) would also work.
    
    # Draw the border
    t.pencolor('black')
    t.pendown()
    t.pensize(10)
    for distance in (screen_x, screen_y, screen_x, screen_y):
        t.forward(distance)
        t.right(90)

    # Raise the pen and move the turtle home again; it's a good idea
    # to leave the turtle in a known state.
    t.penup()
    t.home()

def ver_obstacle(t, obs_x, obs_y, screen_x, screen_y):
	t.penup()
	t.home()

	t.goto(obs_x,obs_y)
	t.right(90)
	t.forward(ver_block_height/2)
	t.right(90)
	t.forward(ver_block_widht/2)
	t.right(90)

	t.pencolor('black')
	t.pendown()
	t.pensize(1)
	t.forward(ver_block_height)
	t.right(90)
	t.forward(ver_block_widht)
	t.right(90)
	t.forward(ver_block_height)
	t.right(90)
	t.forward(ver_block_widht)

	t.penup()
	t.home()

def hor_obstacle(t, obs_x, obs_y, screen_x, screen_y):
	t.penup()
	t.home()

	t.goto(obs_x,obs_y)
	t.right(90)
	t.forward(hor_block_height/2)
	t.right(90)
	t.forward(hor_block_widht/2)
	t.right(90)

	t.pencolor('black')
	t.pendown()
	t.pensize(1)
	t.forward(hor_block_height)
	t.right(90)
	t.forward(hor_block_widht)
	t.right(90)
	t.forward(hor_block_height)
	t.right(90)
	t.forward(hor_block_widht)

	t.penup()
	t.home()

def drawLine(t, p1_x, p1_y, p2_x, p2_y, color, size):
	p1_x = p1_x * 20
	p1_y = p1_y * 20
	p2_x = p2_x * 20
	p2_y = p2_y * 20
	t.goto(p1_x,p1_y)

	t.pencolor(color)
	t.pendown()
	t.pensize(size)
	t.goto(p2_x,p2_y)
	t.penup()
	
def main():
	obstacle_map = []
	graph_file_name = ""
	path_file_name = ""
	map_id = 1
	if len(sys.argv) < 4:
	    print("prm1 : graph.txt \n prm2 : path.txt \n prm3 : map_id  (1,2,3)")
	    exit()    
	else:
		graph_file_name = sys.argv[1]
		path_file_name = sys.argv[2]
		map_id = int(sys.argv[3])
	#Select Map
	if map_id == 1:
		obstacle_map = map1_obstacles
	elif map_id == 2:
		obstacle_map = map2_obstacles
	else:
		obstacle_map = map3_obstacles

	# Create screen and turtle.
	turtle.setup(600,600)
	screen = turtle.Screen()
	screen.title('Visualization for Motion Planning')
	#screen_x, screen_y = screen.screensize()
	screen_x = 400
	screen_y = 400
	t = turtle.Turtle()

	# Uncomment to draw the graphics as quickly as possible.
	t.speed(0)

	# Draw a border around the canvas
	border(t, screen_x, screen_y)

	for obstacle in obstacle_map:
		is_hor = obstacle[0]
		if is_hor == 1:
			hor_obstacle(t, obstacle[1]*20, obstacle[2]*20, screen_x, screen_y)
		else:
			ver_obstacle(t, obstacle[1]*20, obstacle[2]*20, screen_x, screen_y)
	readGraph(t, graph_file_name)
	readPath(t, path_file_name)

	# Draw a set of nested squares, varying the color.
	# The squares are 10%, 20%, etc. of half the size of the canvas.
	#colors = ['red', 'orange', 'yellow', 'green', 'blue', 'violet']
	#t.pensize(3)
	#for i, color in enumerate(colors):
	#    square(t, (screen_y / 2) / 10 * (i+1), color)

	print('Hit any key to exit')
	dummy = input()
        
if __name__ == '__main__':
	main()

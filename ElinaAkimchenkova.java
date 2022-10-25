
import java.io.*;
import java.util.*;
import static java.lang.Math.*;


/*
  @author Elina Akimchenkova, BS21 -07, e.akimchenkova@innopolis.university
  Introduction to Artificial Intelligence, Winter Semester 2022
 */

/**
 * Main class
 */
public class ElinaAkimchenkova {
    /**
     * Input reading, validation and program execution
     *
     * @param args command line arguments that are not used in this task
     * @throws IOException file does not exist
     */
    public static void main(String[] args) throws IOException {

        //Selection input from a file or generate randomly
        Scanner input = new Scanner(System.in);
        System.out.println("""
                Chose one:
                 1. To generate the map and manually insert perception scenario from console
                 2. To insert the positions of agents and perception scenario from the input.txt""");
        int num = input.nextInt();
        //Input validation
        while (!(num == 1 || num == 2)) {
            System.out.println("Please, choose 1 or 2 variant!");
            num = input.nextInt();
        }
        int scenario;
        if (num == 1) {
            //Scenario selection and validation
            try {
                scenario = input.nextInt();
                if (scenario != 1 && scenario != 2) {
                    throw new InputMismatchException();
                }
            } catch (InputMismatchException e) {
                System.out.println("Incorrect scenario");
                return;
            }

            Heroes board = new Heroes();
            AStar.solveProblem(board, scenario);
            BackTracking.solveProblem(board, scenario);
        } else {
            //Input from a file
            BufferedReader reader;
            try {
                reader = new BufferedReader(new FileReader("input.txt"));
            } catch (IOException e) {
                System.out.println("input.txt does not exist!");
                return;
            }
            String line = reader.readLine();
            // Input validation through regular expression
            boolean res = line.matches("^\\[[0],[0]]\\s(\\[\\d,\\d]\\s){4}\\[\\d,\\d]$");
            if (!res) {
                System.out.println("Incorrect input");
                return;
            }

            //Scenario selection and validation
            try {
                scenario = Integer.parseInt(reader.readLine());
                if (scenario != 1 && scenario != 2) {
                    throw new NumberFormatException();
                }
            } catch (NumberFormatException e) {
                System.out.println("Incorrect scenario");
                return;
            }

            //Checking input for number of lines
            try {
                reader.readLine();
            } catch (Exception e) {
                System.out.println("Incorrect input, should be two lines!");
                return;
            }

            //Parsing of input
            ArrayList<Algorithm.Pair> pairs = new ArrayList<>();
            String[] splitLine = line.split(" ");
            for (String s : splitLine) {
                int parseInt1 = Integer.parseInt(String.valueOf(s.charAt(1)));
                int parseInt2 = Integer.parseInt(String.valueOf(s.charAt(3)));
                pairs.add(new Algorithm.Pair(parseInt1, parseInt2));
            }
            Heroes heroes = new Heroes(pairs);

            /*
              Checking restrictions on the coordinates
              If everything is okay, solve problem, otherwise incorrect input
             */
            if (heroes.validityCoordinates(heroes)) {
                AStar.solveProblem(heroes, scenario);
                BackTracking.solveProblem(heroes, scenario);
            } else {
                System.out.println("Incorrect input");
            }
        }
    }
}

/**
 * Interface for checking if hero in visible zone of Kraken, Davy Jones,
 * Tortuga, DeadMansChest, Rock
 */
interface AbilityVisible {
    /**
     * @param xJack - x - coordinates of Jack
     * @param yJack - y - coordinates of Jack
     * @return True if Jack in visible zone, else False
     */
    boolean visible(int xJack, int yJack);
}

/**
 * Class Const introduce constant variables
 */
class Const {
    /**
     * large number for fill cells
     */
    static int INF = (int) 1e9;
}

/**
 * Class Character for saving coordinates
 */
abstract class Character {
    /**
     * x - coordinate
     */
    int x;

    /**
     * y - coordinate
     */
    int y;

    /**
     * Constructor of class
     *
     * @param x - x - coordinate
     * @param y - y - coordinate
     */
    public Character(int x, int y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return x - coordinate
     */
    int getX() {
        return x;
    }

    /**
     * @return y - coordinate
     */
    int getY() {
        return y;
    }
}

/**
 * Class that makes object of class JackSparrow
 */
class JackSparrow extends Character {
    /**
     * Constructor of the class
     *
     * @param x - x - coordinate of Jack
     * @param y - y - coordinate of Jack
     */
    JackSparrow(int x, int y) {
        super(x, y);
    }
}

/**
 * Class that makes object of class DavyJones
 */
class DavyJones extends Character implements AbilityVisible {

    /**
     * Constructor of the class
     *
     * @param x - x - coordinate of Davy Jones
     * @param y - y - coordinate of Davy Jones
     */
    DavyJones(int x, int y) {
        super(x, y);
    }

    /**
     * visible() checks if Jack in danger zone of Davy Jones
     *
     * @param xJack - x - coordinates of Jack
     * @param yJack - y - coordinates of Jack
     * @return True if actor in danger zone, otherwise false
     */
    public boolean visible(int xJack, int yJack) {
        int[][] coord = {{0, 0}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}};
        for (int[] ints : coord) {
            if ((x + ints[0] < 0) || (y + ints[1] < 0) || (x + ints[0] > 9) || (y + ints[1] > 9)) {
                continue;
            }
            if (xJack == x + ints[0] && yJack == y + ints[1]) {
                return true;
            }
        }
        return false;
    }
}

/**
 * Class that makes object of class Kraken
 */
class Kraken extends Character implements AbilityVisible {

    /**
     * Constructor of the class
     *
     * @param x - x - coordinate of Kraken
     * @param y - y - coordinate of Kraken
     */
    Kraken(int x, int y) {
        super(x, y);
    }

    /**
     * visible() checks if Jack in danger zone of Kraken
     *
     * @param xJack - x - coordinates of Jack
     * @param yJack - y - coordinates of Jack
     * @return True if Jack in danger zone, otherwise false
     */
    public boolean visible(int xJack, int yJack) {
        int[][] coord = {{0, 0}, {-1, 0}, {0, -1}, {1, 0}, {0, 1}};
        for (int[] ints : coord) {
            if ((x + ints[0] < 0) || (y + ints[1] < 0) || (x + ints[0] > 9) || (y + ints[1] > 9)) {
                continue;
            }
            if (xJack == x + ints[0] && yJack == y + ints[1]) {
                return true;
            }
        }
        return false;
    }
}

/**
 * Class that makes object of class Rock
 */
class Rock extends Character implements AbilityVisible {

    /**
     * Constructor of the class
     *
     * @param x - x - coordinate of Rock
     * @param y - y - coordinate of Rock
     */
    Rock(int x, int y) {
        super(x, y);
    }

    /**
     * visible() checks if Jack in visible zone
     *
     * @param xJack - x - coordinates of Jack
     * @param yJack - y - coordinates of Jack
     * @return True if actor in visible zone of Rock, otherwise false
     */
    public boolean visible(int xJack, int yJack) {
        return xJack == x && yJack == y;
    }
}

/**
 * Class that makes object of class DeadMansChest
 */
class DeadMansChest extends Character implements AbilityVisible {

    /**
     * Constructor of the class
     *
     * @param x - x - coordinate of DeadMansChest
     * @param y - y - coordinate of DeadMansChest
     */
    DeadMansChest(int x, int y) {
        super(x, y);
    }

    /**
     * visible() checks if Jack can access Dead Mans Chest in one step
     *
     * @param xJack - x - coordinates of Jack
     * @param yJack - y - coordinates of Jack
     * @return True if Jack can, otherwise false
     */
    public boolean visible(int xJack, int yJack) {
        return xJack == x && yJack == y;
    }
}

/**
 * Class that makes object of class Tortuga
 */
class Tortuga extends Character implements AbilityVisible {

    /**
     * Constructor of the class
     *
     * @param x - x - coordinate of Tortuga
     * @param y - y - coordinate of Tortuga
     */
    Tortuga(int x, int y) {
        super(x, y);
    }

    /**
     * visible() checks if Jack can access Tortuga in one step
     *
     * @param xJack - x - coordinates of Jack
     * @param yJack - y - coordinates of Jack
     * @return True if Jack can, otherwise false
     */
    public boolean visible(int xJack, int yJack) {
        return xJack == x && yJack == y;
    }
}

/**
 * Class that sane the current state of the situation (position coordinates, path
 * length, boolean flag is kraken alive and does Jack have tortuga)
 */
class State {
    /**
     * the flag by which it is known whether the tortuga has reached
     */
    boolean haveTortuga;

    /**
     * x - coordinate
     */
    int x;

    /**
     * y - coordinate
     */
    int y;

    /**
     * the flag by which the kraken is known to have lived
     */
    boolean krakenAlive;

    /**
     * length of the path from starting position
     */
    int pathLength = 0;

    /**
     * Class constructor
     *
     * @param x           - current x - coordinate
     * @param y           - current y - coordinate
     * @param haveTortuga - flag if Jack has tortuga
     * @param krakenAlive - flag if Kraken alive
     */
    State(int x, int y, boolean haveTortuga, boolean krakenAlive) {
        this.haveTortuga = haveTortuga;
        this.krakenAlive = krakenAlive;
        this.x = x;
        this.y = y;
    }

    /**
     * Class constructor
     *
     * @param x           - current x - coordinate
     * @param y           - current y - coordinate
     * @param haveTortuga - flag if Jack has tortuga
     * @param krakenAlive - flag if Kraken alive
     * @param pathLength  - path length to current cell
     */
    State(int x, int y, boolean haveTortuga, boolean krakenAlive, int pathLength) {
        this.haveTortuga = haveTortuga;
        this.krakenAlive = krakenAlive;
        this.x = x;
        this.y = y;
        this.pathLength = pathLength;
    }

    /**
     * Return copy of state
     *
     * @return state
     */
    State copy() {
        return new State(x, y, haveTortuga, krakenAlive, pathLength);
    }
}

/**
 * Class Algorithm general for both algorithms (A* and Backtracking).
 * It includes potential movements of Jack (coord) and the minimum number of steps
 * to take from the initial Jack position to the current cell (minMap)
 */
abstract class Algorithm {
    /**
     * x - coordinate
     */
    int x;

    /**
     * y - coordinate
     */
    int y;

    /**
     * Each 2D mass cell of minMap holds the minimum number of steps to be taken from the starting point
     */
    int[][] minMap;

    /**
     * Array of possible displacement
     */
    int[][] coord = {{-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}};
    /**
     * Array of possible displacement for scenario 2
     */
    int[][] coordScenario2 = {{-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-2, 0}, {2, 0}, {0, 2}, {0, -2}};

    /**
     * Contains path coordinates
     */
    ArrayList<Algorithm.Pair> path;

    /**
     * Length of the path
     */
    int pathLength;

    /**
     * Scenario of the game
     */
    int scenario;

    /**
     * Class constructor. Initially, fill all minMap cells with large numbers
     *
     * @param scenario - scenario of the game (first or second)
     * @param x        x - coordinate
     * @param y        y - coordinate
     */
    Algorithm(int x, int y, int scenario) {
        this.scenario = scenario;
        this.x = x;
        this.y = y;
        this.minMap = new int[x][y];
        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                minMap[i][j] = Const.INF;
            }
        }
    }

    /**
     * validCoordinates() checks the validity of coordinates,
     * whether we have not left the playing field
     *
     * @param x - x- coordinate
     * @param y - y - coordinate
     * @return True if coordinates are valid otherwise false
     */
    public boolean validCoordinates(int x, int y) {
        return (0 <= x && x < 9) && (0 <= y && y < 9);
    }

    /**
     * Class that makes a node for Priority Queue for A* algorithm.
     * Contain current state, heuristic distance (vertical/horizontal and diagonal
     * costs are the same), distance from start and sum of them
     */
    static class QueueData implements Comparable {
        /**
         * distance from start position
         */
        int fromStart;

        /**
         * heuristic distance to finish
         */
        int heuristic;

        /**
         * sum of heuristic distance and distance from start
         */
        int sum;

        /**
         * current state
         */
        State state;

        /**
         * Class constructor
         *
         * @param h  - heuristic distance
         * @param st - current state
         */
        QueueData(int h, State st) {
            this.fromStart = st.pathLength;
            this.heuristic = h;
            this.sum = st.pathLength + h;
            this.state = st;
        }

        @Override
        public int compareTo(Object o) {
            QueueData q = (QueueData) o;
            if (sum != q.sum)
                return sum - q.sum;
            return heuristic - q.heuristic;
        }
    }

    /**
     * Class Pair contain pair coordinates
     */
    static class Pair {
        /**
         * x - coordinate
         */
        int x;

        /**
         * y - coordinate
         */
        int y;

        /**
         * Class constructor
         *
         * @param x - x - coordinate
         * @param y - y - coordinate
         */
        Pair(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * getPaths() finds the shortest path from starting position to final position
     *
     * @param xStart - starting x - position
     * @param yStart - starting y - position
     * @param xFinal - final x - position
     * @param yFinal - final y - position
     * @param coord  - array of possible displacement
     * @return ArrayList of pairs of coordinates
     */
    public ArrayList<Pair> getPaths(int xStart, int yStart, int xFinal, int yFinal, int[][] coord) {
        ArrayList<Pair> path = new ArrayList<>();
        Pair finalPair = new Pair(xFinal, yFinal);
        path.add(finalPair);
        while (xStart != xFinal || yStart != yFinal) {
            for (int[] ints : coord) {
                if (!validCoordinates(xFinal + ints[0], yFinal + ints[1])) {
                    continue;
                }
                if (minMap[xFinal + ints[0]][yFinal + ints[1]] < minMap[xFinal][yFinal]) {
                    Pair newPair = new Pair(xFinal + ints[0], yFinal + ints[1]);
                    path.add(0, newPair);
                    xFinal = xFinal + ints[0];
                    yFinal = yFinal + ints[1];
                    break;
                }
            }
        }
        return path;
    }

    /**
     * drawPath() visually shows the shortest path from the starting cell to the final
     *
     * @param path - ArrayList of pairs of coordinates
     * @return array, which includes "*" where actor have gone
     */
    static public char[][] drawPath(ArrayList<Pair> path) {
        char[][] drawing = new char[9][9];
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9; j++) {
                drawing[i][j] = '-';
            }
        }
        for (Pair pair : path) {
            drawing[pair.x][pair.y] = '*';
        }
        return drawing;
    }

    /**
     * drawOutput() print output with information of the current game, such as
     * Win/Lose, in case of win the shortest path length till the Dead Man’s Chest,
     * the shortest path sequence, e.g. [0,0] [1,1] [1,2]...[8,7], the path highlighted
     * on the 2D map and time taken by the algorithm to reach the Dead Man’s Chest, e.g. 100 ms (in ms)
     *
     * @param dirPath     - path if actor have gone directly without tortuga
     * @param tortugaPath - path to the tortuga
     * @param finalPath   - path from the tortuga to the Dead Man’s Chest
     * @param ansTime     - time taken by the algorithm
     */
    static void drawOutput(Algorithm dirPath, Algorithm tortugaPath, Algorithm finalPath, long ansTime) {
        FileWriter output;
        try {
            if (dirPath instanceof AStar) {
                output = new FileWriter("outputAStar.txt", false);
            } else {
                output = new FileWriter("outputBacktracking.txt", false);
            }

            if (tortugaPath.pathLength + finalPath.pathLength > Const.INF) {
                output.write("Lose\n");
                output.close();
                return;
            }
            output.write("Win\n");
            ArrayList<Algorithm.Pair> shortestPaths;
            if (dirPath.pathLength < tortugaPath.pathLength + finalPath.pathLength) {
                output.write(String.valueOf(dirPath.pathLength));
                output.write("\n");
                shortestPaths = dirPath.path;

            } else {
                output.write(String.valueOf(tortugaPath.pathLength + finalPath.pathLength));
                output.write("\n");
                shortestPaths = tortugaPath.path;
                shortestPaths.addAll(finalPath.path);
            }

            for (Algorithm.Pair pair : shortestPaths) {
                output.write("[" + pair.x + "," + pair.y + "] ");
            }
            output.write("\n");
            output.write("-------------------");
            output.write("\n");
            output.write("  ");
            for (int i = 0; i < 9; i++) {
                output.write(i + " ");
            }
            output.write("\n");

            char[][] draw = drawPath(shortestPaths);
            for (int i = 0; i < 9; i++) {
                output.write(i + " ");
                for (int j = 0; j < 9; j++) {
                    output.write(draw[i][j] + " ");
                }
                output.write("\n");
            }
            output.write("-------------------");
            output.write("\n");
            output.write((double) ansTime / 1000000 + " ms");
            output.write("\n");
            output.close();
        } catch (IOException e) {
            System.out.println("Error!");
        }
    }
}

/**
 * Class for implementation A* algorithm
 */
class AStar extends Algorithm {

    /**
     * solveProblem() - function that solve problem in two ways and chose one best:
     * in case of direct path, when kraken alive and actor have no tortuga and go directly to the DeadMansChest
     * in case of sum of two path (path to the tortuga, path from tortuga to DeadMansChest with killing Kraken)
     *
     * @param heroes - array of all objects on playing board
     */
    static void solveProblem(Heroes heroes, int scenario) {
        JackSparrow jackSparrow = heroes.getJackSparrow();
        Tortuga tortuga = heroes.getTortuga();
        DeadMansChest deadMansChest = heroes.getDeadMansChest();

        long beginTime = System.nanoTime();
        // direct
        AStar dirPath = new AStar(scenario);
        dirPath.solve(jackSparrow.getX(), jackSparrow.getY(), deadMansChest.getX(), deadMansChest.getY(), heroes.getEnemies(), tortuga);

        // through Tortuga
        AStar tortugaPath = new AStar(scenario);
        tortugaPath.solve(jackSparrow.getX(), jackSparrow.getY(), tortuga.getX(), tortuga.getY(), heroes.getEnemies(), tortuga);
        AStar finalPath = new AStar(scenario);
        finalPath.solve(tortuga.getX(), tortuga.getY(), deadMansChest.getX(), deadMansChest.getY(), heroes.getEnemies(), tortuga);
        long ansTime = System.nanoTime() - beginTime;
        drawOutput(dirPath, tortugaPath, finalPath, ansTime);

    }

    /**
     * solve() - caller function aStar(), that solve problem
     *
     * @param xStart  - starting x - position
     * @param yStart  - starting y - position
     * @param xFinal  - final x - position
     * @param yFinal  - final y - position
     * @param enemies - array which contain Davy Jones, kraken and rock
     * @param tortuga - tortuga's position of the map
     */
    void solve(int xStart, int yStart, int xFinal, int yFinal, AbilityVisible[] enemies, Tortuga tortuga) {
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9; j++) {
                minMap[i][j] = Const.INF;
            }
        }
        State beginState = new State(xStart, yStart, false, true);
        if (scenario == 1) {
            aStar(xStart, yStart, xFinal, yFinal, beginState, coord, enemies, tortuga);
        } else {
            aStar(xStart, yStart, xFinal, yFinal, beginState, coordScenario2, enemies, tortuga);
        }
        pathLength = minMap[xFinal][yFinal];
        if (minMap[xFinal][yFinal] != Const.INF) {
            path = getPaths(xStart, yStart, xFinal, yFinal, coord);
        }
    }

    /**
     * Class Constructor
     */
    AStar(int scenario) {
        super(9, 9, scenario);
    }

    /**
     * heuristic() - function that computes heuristic distance from current point to final
     *
     * @param xStart - current x - position
     * @param yStart - current y - position
     * @param xFinal - final x - position
     * @param yFinal - final y - position
     * @return heuristic distance from current point to final
     */
    int heuristic(int xStart, int yStart, int xFinal, int yFinal) {
        return min(abs(xStart - xFinal), abs(yStart - yFinal)) + abs((abs(xStart - xFinal) - abs(yStart - yFinal)));
    }


    /**
     * aStar() - implementation of A* algorithm
     *
     * @param xStart  - current x - position
     * @param yStart  - current y - position
     * @param xFinal  - final x - position
     * @param yFinal  - final y - position
     * @param state   - current state
     * @param coord   - array of possible movements
     * @param enemies - array which include Davy Jones, Kraken and Rock
     * @param tortuga - position of the tortuga on the playing ground
     */
    void aStar(int xStart, int yStart, int xFinal, int yFinal, State state, int[][] coord, AbilityVisible[] enemies, Tortuga tortuga) {
        PriorityQueue<QueueData> queue = new PriorityQueue<>();

        Kraken kraken = (Kraken) enemies[1];
        minMap[xStart][yStart] = 0;
        queue.add(new QueueData(heuristic(state.x, state.y, xFinal, yFinal), state));
        int step;
        if (scenario == 2) {
            step = coord.length - 4;
        } else {
            step = coord.length;
        }

        while (!queue.isEmpty()) {
            QueueData newQueueData = queue.poll();
            int xCurrent = newQueueData.state.x;
            int yCurrent = newQueueData.state.y;

            if (tortuga.visible(xCurrent, yCurrent)) {
                newQueueData.state.haveTortuga = true;
            }
            if (newQueueData.state.haveTortuga && newQueueData.state.krakenAlive) {
                for (int i = 0; i < step; i++) {
                    if (xCurrent + coord[i][0] == kraken.getX() && yCurrent + coord[i][1] == kraken.getY()) {
                        newQueueData.state.krakenAlive = false;
                        break;
                    }
                }
            }

            if (xFinal == xCurrent && yFinal == yCurrent) {
                return;
            }
            for (int[] ints : coord) {
                int xMoved = newQueueData.state.x + ints[0];
                int yMoved = newQueueData.state.y + ints[1];
                if (!validCoordinates(xMoved, yMoved)) {
                    continue;
                }
                boolean flag = true;
                int inc = 0;
                if (abs(ints[0]) == 2 || abs(ints[1]) == 2) {
                    int xTemp = newQueueData.state.x + ints[0] / 2;
                    int yTemp = newQueueData.state.y + ints[1] / 2;
                    for (int k = 0; k < 3; k++) {
                        if (enemies[k].visible(xTemp, yTemp) && !(enemies[1].visible(xTemp, yTemp) && !newQueueData.state.krakenAlive)) {
                            flag = false;
                            break;
                        }
                    }
                    inc = 1;
                }
                if (!flag) {
                    continue;
                }
                for (int k = 0; k < 3; k++) {
                    if (enemies[k].visible(xMoved, yMoved) && !(enemies[1].visible(xMoved, yMoved) && !newQueueData.state.krakenAlive)) {
                        flag = false;
                        break;
                    }
                }

                if (flag && minMap[xMoved][yMoved] > newQueueData.fromStart + 1 + inc) {
                    minMap[xMoved][yMoved] = newQueueData.fromStart + 1 + inc;
                    State stateMove = newQueueData.state.copy();
                    stateMove.x = xMoved;
                    stateMove.y = yMoved;
                    stateMove.pathLength += inc + 1;
                    queue.add(new QueueData(heuristic(xMoved, yMoved, xFinal, yFinal), stateMove));
                }
            }
        }
    }
}

/**
 * Class for implementation BackTracking algorithm
 */
class BackTracking extends Algorithm {

    /**
     * solveProblem() - function that solve problem in two ways and chose one best:
     * in case of direct path, when kraken alive and actor have no tortuga and go directly to the DeadMansChest
     * in case of sum of two path (path to the tortuga, path from tortuga to DeadMansChest with killing Kraken)
     *
     * @param heroes - array of all objects on playing board
     */
    static void solveProblem(Heroes heroes, int scenario) {
        JackSparrow jackSparrow = heroes.getJackSparrow();
        Tortuga tortuga = heroes.getTortuga();
        DeadMansChest deadMansChest = heroes.getDeadMansChest();

        long beginTime = System.nanoTime();
        // direct
        BackTracking dirPath = new BackTracking(scenario);
        dirPath.solve(jackSparrow.getX(), jackSparrow.getY(), deadMansChest.getX(), deadMansChest.getY(), heroes.getEnemies(), tortuga);

        // through Tortuga
        BackTracking tortugaPath = new BackTracking(scenario);
        tortugaPath.solve(jackSparrow.getX(), jackSparrow.getY(), tortuga.getX(), tortuga.getY(), heroes.getEnemies(), tortuga);
        BackTracking finalPath = new BackTracking(scenario);
        finalPath.solve(tortuga.getX(), tortuga.getY(), deadMansChest.getX(), deadMansChest.getY(), heroes.getEnemies(), tortuga);
        long ansTime = System.nanoTime() - beginTime;
        drawOutput(dirPath, tortugaPath, finalPath, ansTime);
    }

    /**
     * solve() - caller function backtrack(), that solve problem
     *
     * @param xStart  - starting x - position
     * @param yStart  - starting y - position
     * @param xFinal  - final x - position
     * @param yFinal  - final y - position
     * @param enemies - array which contain Davy Jones, kraken and rock
     * @param tortuga - tortuga's position of the map
     */
    void solve(int xStart, int yStart, int xFinal, int yFinal, AbilityVisible[] enemies, Tortuga tortuga) {
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9; j++) {
                minMap[i][j] = Const.INF;
            }
        }
        State beginState = new State(xStart, yStart, false, true);
        if (scenario == 1) {
            backtrack(xStart, yStart, xFinal, yFinal, beginState, coord, enemies, tortuga);
        } else {
            backtrack(xStart, yStart, xFinal, yFinal, beginState, coordScenario2, enemies, tortuga);
        }
        pathLength = minMap[xFinal][yFinal];
        if (minMap[xFinal][yFinal] != Const.INF) {
            path = getPaths(xStart, yStart, xFinal, yFinal, coord);
        }
    }

    /**
     * Class constructor
     */
    BackTracking(int scenario) {
        super(9, 9, scenario);
    }

    /**
     * backtrack() - implementation of BackTracking algorithm
     *
     * @param xStart  - current x - position
     * @param yStart  - current y - position
     * @param xFinal  - final x - position
     * @param yFinal  - final y - position
     * @param state   - current state
     * @param coord   - array of possible movements
     * @param enemies - array which include Davy Jones, Kraken and Rock
     * @param tortuga - position of the tortuga on the playing ground
     */
    void backtrack(int xStart, int yStart, int xFinal, int yFinal, State state, int[][] coord, AbilityVisible[] enemies, Tortuga tortuga) {
        if (minMap[state.x][state.y] > state.pathLength) {
            minMap[state.x][state.y] = state.pathLength;
        } else {
            return;
        }

        int step;
        if (scenario == 2) {
            step = coord.length - 4;
        } else {
            step = coord.length;
        }

        if (tortuga.visible(xStart, yStart)) {
            state.haveTortuga = true;
        }
        if (xFinal == xStart && yFinal == yStart) {
            return;
        }
        if (state.haveTortuga && state.krakenAlive) {
            Kraken kraken = (Kraken) enemies[1];
            for (int i = 0; i < step; i++) {
                if (xStart + coord[i][0] == kraken.getX() && yStart + coord[i][1] == kraken.getY()) {
                    state.krakenAlive = false;
                    break;
                }
            }
        }

        for (int[] ints : coord) {
            int xMoved = state.x + ints[0];
            int yMoved = state.y + ints[1];
            if (!validCoordinates(xMoved, yMoved)) {
                continue;
            }
            boolean flag = true;
            int inc = 0;
            if (abs(ints[0]) == 2 || abs(ints[1]) == 2) {
                int xTemp = state.x + ints[0] / 2;
                int yTemp = state.y + ints[1] / 2;
                for (int k = 0; k < 3; k++) {
                    if (enemies[k].visible(xTemp, yTemp) && !(enemies[1].visible(xTemp, yTemp) && !state.krakenAlive)) {
                        flag = false;
                        break;
                    }
                }
                inc = 1;
            }
            if (!flag) {
                continue;
            }
            for (int k = 0; k < 3; k++) {
                if (enemies[k].visible(xMoved, yMoved) && !(enemies[1].visible(xMoved, yMoved) && !state.krakenAlive)) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                State stateMove = state.copy();
                stateMove.x = xMoved;
                stateMove.y = yMoved;
                stateMove.pathLength += inc + 1;
                backtrack(xMoved, yMoved, xFinal, yFinal, stateMove, coord, enemies, tortuga);
            }
        }
    }
}

/**
 * Class for or objects on playing board
 */
class Heroes {
    /**
     * object of class JackSparrow
     */
    private JackSparrow jackSparrow;

    /**
     * object of class DavyJones
     */
    private DavyJones davyJones;

    /**
     * object of class Kraken
     */
    private Kraken kraken;

    /**
     * object of class Rock
     */
    private Rock rock;

    /**
     * object of class DeadMansChest
     */
    private DeadMansChest deadMansChest;

    /**
     * object of class Tortuga
     */
    private Tortuga tortuga;

    /**
     * Class constructor
     *
     * @param arr - ArrayList of pairs of coordinates of each object on playing board
     */
    Heroes(ArrayList<Algorithm.Pair> arr) {
        jackSparrow = new JackSparrow(arr.get(0).x, arr.get(0).y);
        davyJones = new DavyJones(arr.get(1).x, arr.get(1).y);
        kraken = new Kraken(arr.get(2).x, arr.get(2).y);
        rock = new Rock(arr.get(3).x, arr.get(3).y);
        deadMansChest = new DeadMansChest(arr.get(4).x, arr.get(4).y);
        tortuga = new Tortuga(arr.get(5).x, arr.get(5).y);
    }

    /**
     * Class constructor
     */
    Heroes() {
        RandGenerator();
    }

    /**
     * getAllHeroes() getter for all heroes
     *
     * @return array of positions of each hero
     */
    Character[] getAllHeroes() {
        return new Character[]{davyJones, kraken, rock, deadMansChest, tortuga};
    }

    /**
     * getEnemies() - function produces array of enemies
     *
     * @return array of enemies
     */
    AbilityVisible[] getEnemies() {
        return new AbilityVisible[]{davyJones, kraken, rock};
    }

    /**
     * getJackSparrow() getter for jackSparrow
     *
     * @return jackSparrow
     */
    JackSparrow getJackSparrow() {
        return jackSparrow;
    }

    /**
     * getTortuga() getter for tortuga
     *
     * @return tortuga
     */
    Tortuga getTortuga() {
        return tortuga;
    }

    /**
     * getDeadMansChest() getter for Dead Mans Chest
     *
     * @return deadMansChest
     */
    DeadMansChest getDeadMansChest() {
        return deadMansChest;
    }

    /**
     * getRandInt() - function that generates random integer number in range [0, 8]
     *
     * @return random integer number
     */
    int getRandInt() {
        Random random = new Random();
        return random.nextInt(9);
    }

    /**
     * RandGenerator() generate random map with restriction for each object
     */
    void RandGenerator() {
        jackSparrow = new JackSparrow(0, 0);
        davyJones = new DavyJones(getRandInt(), getRandInt());
        tortuga = new Tortuga(getRandInt(), getRandInt());
        kraken = new Kraken(getRandInt(), getRandInt());
        rock = new Rock(getRandInt(), getRandInt());
        deadMansChest = new DeadMansChest(getRandInt(), getRandInt());

        while (davyJones.getX() == 0 && davyJones.getY() == 0) {
            davyJones = new DavyJones(getRandInt(), getRandInt());
        }

        while ((kraken.getX() == 0 && kraken.getY() == 0) ||
                (davyJones.getX() == kraken.getX() && davyJones.getY() == kraken.getY())) {
            kraken = new Kraken(getRandInt(), getRandInt());
        }
        while ((rock.getX() == 0 && rock.getY() == 0) ||
                (davyJones.getX() == rock.getX() && davyJones.getY() == rock.getY()) ||
                (tortuga.getX() == rock.getX() && tortuga.getY() == rock.getY()) ||
                (deadMansChest.getX() == rock.getX() && deadMansChest.getY() == rock.getY())) {
            rock = new Rock(getRandInt(), getRandInt());
        }

        while ((deadMansChest.getX() == 0 && deadMansChest.getY() == 0) ||
                (davyJones.visible(deadMansChest.getX(), deadMansChest.getY())) ||
                (deadMansChest.getX() == davyJones.getX() && deadMansChest.getY() == davyJones.getY()) ||
                (kraken.visible(deadMansChest.getX(), deadMansChest.getY()) || (deadMansChest.getX() ==
                        kraken.getX() && deadMansChest.getY() == kraken.getY()))) {
            deadMansChest = new DeadMansChest(getRandInt(), getRandInt());
        }

        while ((davyJones.visible(tortuga.getX(), tortuga.getY())) ||
                (kraken.visible(tortuga.getX(), tortuga.getY()))) {
            tortuga = new Tortuga(getRandInt(), getRandInt());
        }
    }

    /**
     * validityCoordinates() checks validity of the coordinates entered
     *
     * @param arr - array of all objects and their coordinates
     * @return True if coordinates are valid otherwise false
     */
    boolean validityCoordinates(Heroes arr) {
        Character[] obj = getAllHeroes();
        for (Character character : obj) {
            if (!(0 <= character.getX() && character.getX() < 9 && 0 <= character.getY() && character.getY() < 9)) {
                return false;
            }
        }
        if (arr.davyJones.getX() == 0 && arr.davyJones.getY() == 0) {
            return false;
        }

        if ((arr.kraken.getX() == 0 && arr.kraken.getY() == 0) ||
                (arr.davyJones.getX() == arr.kraken.getX() && arr.davyJones.getY() == arr.kraken.getY())) {
            return false;
        }
        if ((arr.rock.getX() == 0 && arr.rock.getY() == 0) ||
                (arr.davyJones.getX() == arr.rock.getX() && arr.davyJones.getY() == arr.rock.getY()) ||
                (arr.tortuga.getX() == arr.rock.getX() && arr.tortuga.getY() == arr.rock.getY()) ||
                (arr.deadMansChest.getX() == arr.rock.getX() && arr.deadMansChest.getY() == arr.rock.getY())) {
            return false;
        }

        if ((arr.deadMansChest.getX() == 0 && arr.deadMansChest.getY() == 0) ||
                (arr.davyJones.visible(arr.deadMansChest.getX(), arr.deadMansChest.getY())) ||
                (arr.deadMansChest.getX() == arr.davyJones.getX() && arr.deadMansChest.getY() == arr.davyJones.getY()) ||
                (arr.kraken.visible(arr.deadMansChest.getX(), arr.deadMansChest.getY()) || (arr.deadMansChest.getX() ==
                        arr.kraken.getX() && arr.deadMansChest.getY() == arr.kraken.getY()))) {
            return false;
        }

        return (!arr.davyJones.visible(arr.tortuga.getX(), arr.tortuga.getY())) &&
                (!arr.kraken.visible(arr.tortuga.getX(), arr.tortuga.getY()));
    }
}

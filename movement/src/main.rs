use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::io::{self, Write};

const MAX_WHEEL_SPEED: f64 = 1.0;
const BASE_SPEED: f64 = 0.7;

const SAFETY_DISTANCE: f64 = 1.0;
const OBSTACLE_CLEAR_DISTANCE: f64 = 1.5;

const POSITION_TOLERANCE: f64 = 0.15;

const TURN_GAIN: f64 = 0.015;

// ---------------------------------------------------------
// BASIC DATA TYPES
// ---------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct Position {
    x: f64,
    y: f64,
}

impl Position {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y}
    }

    fn distance_to(&self, other: &Position) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;

        (dx * dx + dy * dy).sqrt()
    }
}

// ---------------------------------------------------------
// OBJECT DETECTION INPUT
// ---------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
struct ObjectDetection {
    object_position: f64,
    object_angle: f64,
    found: bool,
}

// ---------------------------------------------------------
// MOVEMENT ENUMS
// ---------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize)]
enum Direction {
    Forward,
    Backward,
    Left,
    Right,
    Stop,
}

#[derive(Debug, Clone, Copy, Serialize)]
enum MovementState {
    Idle,
    Moving,
    Turning,
    AvoidingLeft,
    AvoidingRight,
    Stopped,
    Reached,
}

// ---------------------------------------------------------
// MOVEMENT COMMAND
// ---------------------------------------------------------

#[derive(Debug, Serialize)]
struct MovementCommand {
    state: MovementState,
    direction: Direction,
    angle: f64,
    left_velocity: f64,
    right_velocity: f64,
}

// ---------------------------------------------------------
// ROBOT
// ---------------------------------------------------------

struct Robot {
    position: Position,

    // Heading in degrees.
    // 0° = +X direction
    // 90° = +Y direction
    heading: f64,

    state: MovementState,
}

impl Robot {
    fn new(position: Position, heading: f64) -> Self {
        Self {
            position,
            heading: normalize_angle(heading),
            state: MovementState::Idle,
        }
    }

    // -----------------------------------------------------
    // MOVE ROBOT IN SIMULATION
    // -----------------------------------------------------

    fn simulate_motion(
        &mut self,
        left_velocity: f64,
        right_velocity: f64,
        dt: f64,
    ) {
        let linear_velocity = (left_velocity + right_velocity) / 2.0;

        let angular_velocity = right_velocity - left_velocity;

        // Simple heading update.
        self.heading += angular_velocity * 30.0 * dt;
        self.heading = normalize_angle(self.heading);

        let heading_rad = degrees_to_radians(self.heading);

        self.position.x +=
            linear_velocity * heading_rad.cos() * dt;

        self.position.y +=
            linear_velocity * heading_rad.sin() * dt;
    }
}

// ---------------------------------------------------------
// MOVEMENT MANAGER
// ---------------------------------------------------------

struct MovementManager {
    target: Position,

    // Used to remember which side was selected for avoidance.
    avoidance_direction: Option<Direction>,

    // Used when robot is temporarily avoiding an obstacle.
    avoiding: bool,
}

impl MovementManager {
    fn new(target: Position) -> Self {
        Self {
            target,
            avoidance_direction: None,
            avoiding: false,
        }
    }

    // -----------------------------------------------------
    // MAIN DECISION FUNCTION
    // -----------------------------------------------------

    fn update(
        &mut self,
        robot: &mut Robot,
        detection: &ObjectDetection,
    ) -> MovementCommand {
        // ---------------------------------------------
        // 1. Check whether target has been reached
        // ---------------------------------------------

        let distance_to_target =
            robot.position.distance_to(&self.target);

        if distance_to_target <= POSITION_TOLERANCE {
            robot.state = MovementState::Reached;

            return MovementCommand {
                state: MovementState::Reached,
                direction: Direction::Stop,
                angle: 0.0,
                left_velocity: 0.0,
                right_velocity: 0.0,
            };
        }

        // ---------------------------------------------
        // 2. Check obstacle
        // ---------------------------------------------

        if detection.found
            && detection.object_position <= SAFETY_DISTANCE
        {
            return self.handle_obstacle(robot, detection);
        }

        // ---------------------------------------------
        // 3. If currently avoiding and obstacle is clear
        // ---------------------------------------------

        if self.avoiding {
            if !detection.found
                || detection.object_position >= OBSTACLE_CLEAR_DISTANCE
            {
                self.avoiding = false;
                self.avoidance_direction = None;
            }
        }

        // ---------------------------------------------
        // 4. Normal target navigation
        // ---------------------------------------------

        self.move_to_target(robot)
    }

    // -----------------------------------------------------
    // OBSTACLE HANDLING
    // -----------------------------------------------------

    fn handle_obstacle(
        &mut self,
        robot: &mut Robot,
        detection: &ObjectDetection,
    ) -> MovementCommand {
        robot.state = MovementState::Stopped;

        /*
            Safety behavior:

            First stop.

            Then decide whether to avoid left or right.

            Positive object angle:
                obstacle is on one side.

            Negative object angle:
                obstacle is on the other side.

            For this first version we select the
            opposite side of the obstacle.
        */

        let direction = if detection.object_angle >= 0.0 {
            Direction::Right
        } else {
            Direction::Left
        };

        self.avoidance_direction = Some(direction);
        self.avoiding = true;

        match direction {
            Direction::Left => {
                robot.state = MovementState::AvoidingLeft;

                MovementCommand {
                    state: MovementState::AvoidingLeft,
                    direction: Direction::Left,
                    angle: 30.0,
                    left_velocity: 0.35,
                    right_velocity: 0.75,
                }
            }

            Direction::Right => {
                robot.state = MovementState::AvoidingRight;

                MovementCommand {
                    state: MovementState::AvoidingRight,
                    direction: Direction::Right,
                    angle: -30.0,
                    left_velocity: 0.75,
                    right_velocity: 0.35,
                }
            }

            _ => self.stop_command(),
        }
    }

    // -----------------------------------------------------
    // MOVE TOWARD TARGET
    // -----------------------------------------------------

    fn move_to_target(
        &mut self,
        robot: &mut Robot,
    ) -> MovementCommand {
        let dx = self.target.x - robot.position.x;
        let dy = self.target.y - robot.position.y;

        let target_angle =
            radians_to_degrees(dy.atan2(dx));

        let heading_error =
            normalize_angle(target_angle - robot.heading);

        /*
            Proportional steering:

                turn = Kp * heading_error

            Positive turn:
                right wheel faster

            Negative turn:
                left wheel faster
        */

        let turn = TURN_GAIN * heading_error;

        let left_velocity =
            clamp(BASE_SPEED - turn);

        let right_velocity =
            clamp(BASE_SPEED + turn);

        let direction;

        if heading_error > 5.0 {
            direction = Direction::Left;
            robot.state = MovementState::Turning;
        } else if heading_error < -5.0 {
            direction = Direction::Right;
            robot.state = MovementState::Turning;
        } else {
            direction = Direction::Forward;
            robot.state = MovementState::Moving;
        }

        MovementCommand {
            state: robot.state,
            direction,
            angle: heading_error,
            left_velocity,
            right_velocity,
        }
    }

    // -----------------------------------------------------
    // STOP COMMAND
    // -----------------------------------------------------

    fn stop_command(&self) -> MovementCommand {
        MovementCommand {
            state: MovementState::Stopped,
            direction: Direction::Stop,
            angle: 0.0,
            left_velocity: 0.0,
            right_velocity: 0.0,
        }
    }
}

// ---------------------------------------------------------
// MATH FUNCTIONS
// ---------------------------------------------------------

fn degrees_to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.0
}

fn radians_to_degrees(radians: f64) -> f64 {
    radians * 180.0 / PI
}

// Normalize angle to [-180, 180)
fn normalize_angle(mut angle: f64) -> f64 {
    while angle >= 180.0 {
        angle -= 360.0;
    }

    while angle < -180.0 {
        angle += 360.0;
    }

    angle
}

fn clamp(value: f64) -> f64 {
    value.clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
}

// ---------------------------------------------------------
// INPUT FUNCTIONS
// ---------------------------------------------------------

fn read_number(prompt: &str) -> f64 {
    loop {
        print!("{}", prompt);

        io::stdout()
            .flush()
            .expect("Failed to flush stdout");

        let mut input = String::new();

        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read input");

        match input.trim().parse::<f64>() {
            Ok(value) => return value,

            Err(_) => {
                println!("Please enter a valid number.");
            }
        }
    }
}

fn read_position(name: &str) -> Position {
    println!();
    println!("Enter {} position:", name);

    let x = read_number("x = ");
    let y = read_number("y = ");

    Position::new(x, y)
}

// ---------------------------------------------------------
// JSON DETECTION INPUT
// ---------------------------------------------------------

fn read_detection() -> ObjectDetection {
    println!();
    println!("Enter object detection JSON.");

    println!(
        r#"Example:
{{"object_position":2.0,"object_angle":10.0,"found":true}}"#
    );

    print!("JSON: ");

    io::stdout()
        .flush()
        .expect("Failed to flush stdout");

    let mut input = String::new();

    io::stdin()
        .read_line(&mut input)
        .expect("Failed to read JSON");

    match serde_json::from_str::<ObjectDetection>(input.trim()) {
        Ok(data) => data,

        Err(error) => {
            println!("Invalid JSON: {}", error);

            ObjectDetection {
                object_position: 999.0,
                object_angle: 0.0,
                found: false,
            }
        }
    }
}

// ---------------------------------------------------------
// PRINT COMMAND AS JSON
// ---------------------------------------------------------

fn print_command(command: &MovementCommand) {
    match serde_json::to_string_pretty(command) {
        Ok(json) => {
            println!();
            println!("========== MOVEMENT OUTPUT ==========");
            println!("{}", json);
            println!("=====================================");
        }

        Err(error) => {
            println!("Could not serialize command: {}", error);
        }
    }
}

// ---------------------------------------------------------
// MAIN SIMULATION
// ---------------------------------------------------------

fn main() {
    println!("========================================");
    println!("       ARIADNE MOVEMENT MANAGER");
    println!("========================================");

    // ---------------------------------------------
    // INPUT
    // ---------------------------------------------

    let initial_position = read_position("INITIAL");

    let final_position = read_position("FINAL");

    let initial_heading =
        read_number("Initial robot heading in degrees = ");

    // ---------------------------------------------
    // CREATE ROBOT
    // ---------------------------------------------

    let mut robot =
        Robot::new(initial_position, initial_heading);

    // ---------------------------------------------
    // CREATE MOVEMENT MANAGER
    // ---------------------------------------------

    let mut manager =
        MovementManager::new(final_position);

    println!();
    println!("Initial robot:");
    println!(
        "Position: ({:.2}, {:.2})",
        robot.position.x,
        robot.position.y,

    );

    println!("Heading: {:.2}°", robot.heading);

    println!(
        "Target: ({:.2}, {:.2})",
        final_position.x,
        final_position.y
    );

    // ---------------------------------------------
    // SIMULATION
    // ---------------------------------------------

    println!();
    println!("Starting simulation...");
    println!("Press ENTER after every detection update.");
    println!();

    let dt = 0.1;

    for step in 0..1000 {
        println!();
        println!("--------------- STEP {} ---------------", step);

        // -----------------------------------------
        // Detection department input
        // -----------------------------------------

        let detection_json = r#"
    {
        "object_position": 2.0,
        "object_angle": 0.0,
        "found": true
    }
    "#;

    let detection: ObjectDetection =
        serde_json::from_str(detection_json)
            .expect("Invalid JSON");

        // -----------------------------------------
        // Movement decision
        // -----------------------------------------

        let command =
            manager.update(&mut robot, &detection);

        // -----------------------------------------
        // Output
        // -----------------------------------------

        print_command(&command);

        println!(
            "Robot position: ({:.2}, {:.2})",
            robot.position.x,
            robot.position.y
        );

        println!(
            "Robot heading: {:.2}°",
            robot.heading
        );

        println!(
            "Distance to target: {:.2} m",
            robot.position.distance_to(&final_position)
        );

        // -----------------------------------------
        // Target reached
        // -----------------------------------------

        if matches!(
            command.state,
            MovementState::Reached
        ) {
            println!();
            println!("========================================");
            println!("        TARGET REACHED");
            println!("========================================");

            break;
        }

        // -----------------------------------------
        // Simulate wheel movement
        // -----------------------------------------

        robot.simulate_motion(
            command.left_velocity,
            command.right_velocity,
            dt,
        );
    }
}


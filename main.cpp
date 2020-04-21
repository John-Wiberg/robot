#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHRPS.h>

#define Red_min 0.001
#define Red_max 0.3
#define TAIL_SERVO_MIN 500
#define TAIL_SERVO_MAX 2385
#define MINI_SERVO_MIN 586
#define MINI_SERVO_MAX 2500
#define LEVER_SERVO_MIN 806
#define LEVER_SERVO_MAX 2269
#define counts_per_inch 2.777977189
#define PI 3.141592653

//declaring pins
AnalogInputPin CdS_Cell(FEHIO::P2_0);
DigitalInputPin Right_encoder(FEHIO::P0_0);
DigitalInputPin Left_encoder(FEHIO::P1_0);
FEHMotor Right_motor(FEHMotor::Motor3,7.2);
FEHMotor Left_motor(FEHMotor::Motor2,7.2);
FEHServo Tail(FEHServo::Servo7);
FEHServo Mini(FEHServo::Servo0);
FEHServo Lever(FEHServo::Servo3);

//functions to test the robot
void RightDigTest();
void LeftDigTest();
void TestMotors();
void TestEncoding();
void CdSControl();
void Calibration();
void TestCdS();

//functions to navigate the robot
void Forward(float);
void MoveTimedDistance(float);
void Backwards(float);
void TurnTimed90Left();
void TurnTimed90Left2();
void TurnTimed90Right();
void TurnTimed45Left();
void TurnTimed45Right();
void encoder(float);
void encoder_right(float);
void Turn_Right(int,float);
void Turn_Left(int,float);
void Point_Turn_Left(float);
void Point_Turn_Right(float);
void check_x_minus(float);
void check_x_plus(float);
void check_y_minus(float);
void check_y_plus(float);
void check_heading(float);

//functions to perform tasks
void Start(float);
void Tray();
void Burger();
void Ticket();
void ReleaseTray();

int main(void)
{
    RPS.InitializeTouchMenu();
    Lever.SetDegree(180);
    LCD.WriteLine(CdS_Cell.Value());
    while(CdS_Cell.Value()>2)
    {}
    Sleep(1.0);
    Start(CdS_Cell.Value());
    Burger();
    return 0;
}

//Function to count encoder value
void encoder(float a)
{
    int count = 0;
    while(count < a)
    {
    if(Left_encoder.Value() == 0)
    {
        count = count + 1;
        while(Left_encoder.Value() == 0)
        {}
    }
    if(Left_encoder.Value() == 1)
    {
        count = count + 1;
        while(Left_encoder.Value() == 1)
        {}
    }
    LCD.WriteLine(count);
    }
}

//Function to start robot
void Start(float b)
{
    LCD.WriteLine(CdS_Cell.Value());
    if(b>Red_min && b<Red_max)
    {
        Forward(13);
        Turn_Right(0,45);
        check_heading(90);
        Left_motor.SetPercent(75.);
        Right_motor.SetPercent(-75.);
        encoder(counts_per_inch*26);
        Left_motor.Stop();
        Right_motor.Stop();
        Sleep(1.0);
    }
}

void Tray()
{
    Forward(22);
    Turn_Left(0,90);
    Forward(3);
    Turn_Left(0,8);
    Mini.SetDegree(90);
    Sleep(1.0);
    Turn_Right(1,8);
}

void Burger()
{
    Turn_Right(0,45);
    Forward(2);
    Turn_Left(0,45);
    Lever.SetDegree(40);
    Forward(3.75);
    //check_y_plus(50.9);
    Sleep(1.5);
    Lever.SetDegree(108);
    Sleep(1.0);
    Turn_Right(0,20);
    Sleep(1.0);
    Backwards(1.5);
    Turn_Right(0,20);
    Forward(1.5);
    Turn_Left(1,30);
}

void Ticket()
{
    Backwards(24.5);
    Forward(.5);
    Turn_Left(0,90);
    check_heading(272);
    Backwards(5);
    Tail.SetDegree(100);
    Forward(4.5);
    check_y_plus(45.9);
    Turn_Right(0,30);
    Sleep(1.0);
    Turn_Left(1,28);
    Backwards(8);
    Tail.SetDegree(0);

}

//function to turn any angle right
void Turn_Right(int x,float angle)
{
    if (x == 0 )
    {
    float percent = angle/360;
    int distance, Circumference = (2*PI*8);
    distance = percent*Circumference;
    Right_motor.SetPercent(5);
    Left_motor.SetPercent(45);
    encoder(counts_per_inch*distance);
    Right_motor.Stop();
    Left_motor.Stop();
    }
    if (x == 1)//backwards
    {
        float percent = angle/360;
        int distance, Circumference = (2*PI*8);
        distance = percent*Circumference;
        Right_motor.SetPercent(45);
        Left_motor.SetPercent(-5);
        encoder_right(counts_per_inch*distance);
        Right_motor.Stop();
        Left_motor.Stop();
    }

}

//function to turn any angle left
void Turn_Left(int x,float angle)
{
    if (x == 0 )
    {
        float percent = angle/360;
        int distance, Circumference = (2*PI*8);
        distance = percent*Circumference;
        Right_motor.SetPercent(-45);
        Left_motor.SetPercent(-5);
        encoder_right(counts_per_inch*distance);
        Right_motor.Stop();
        Left_motor.Stop();
    }
    if (x == 1)
    {
        float percent = angle/360;
        int distance, Circumference = (2*PI*8);
        distance = percent*Circumference;
        Right_motor.SetPercent(5);
        Left_motor.SetPercent(-45);
        encoder(counts_per_inch*distance);
        Right_motor.Stop();
        Left_motor.Stop();
    }
}

void Point_Turn_Right(float angle)
{
    float percent = angle/360;
    int distance, Circumference = (2*PI*4);
    distance = percent*Circumference;
    Right_motor.SetPercent(30);
    Left_motor.SetPercent(30);
    encoder(counts_per_inch*distance);
    Right_motor.Stop();
    Left_motor.Stop();

}

//function to turn any angle left
void Point_Turn_Left(float angle)
{
    float percent = angle/360;
    int distance, Circumference = (2*PI*4);
    distance = percent*Circumference;
    Right_motor.SetPercent(-30);
    Left_motor.SetPercent(-30);
    encoder_right(counts_per_inch*distance);
    Right_motor.Stop();
    Left_motor.Stop();

}
//function to move the robot x inches
void Forward(float x)
{
    Right_motor.SetPercent(-25);
    Left_motor.SetPercent(25);
    encoder(counts_per_inch*x);
    Right_motor.Stop();
    Left_motor.Stop();
}

//function to move the robot backwards x inches
void Backwards(float x)
{
    Right_motor.SetPercent(25);
    Left_motor.SetPercent(-25);
    encoder(counts_per_inch*x);
    Right_motor.Stop();
    Left_motor.Stop();
}


//function to turn a 90 degree angle to the left based on time - temporary
void TurnTimed90Left()
{
    Right_motor.SetPercent(-45);
    Left_motor.SetPercent(-5);
    Sleep(2.5);
    Right_motor.Stop();
    Left_motor.Stop();

}

void TurnTimed90Left2()
{
    Right_motor.SetPercent(-45);
    Left_motor.SetPercent(-5);
    Sleep(2.35);
    Right_motor.Stop();
    Left_motor.Stop();

}
//function to turn a 90 degree angle to the right based on time - temporary
void TurnTimed90Right()
{
    Right_motor.SetPercent(5);
    Left_motor.SetPercent(45);
    Sleep(2.12);
    Right_motor.Stop();
    Left_motor.Stop();

}


//function to release the tray by moving the mini servo
void ReleaseTray()
{

}

//function to turn a 45 degree angle to the left based on time - temporary
void TurnTimed45Left()
{
    Right_motor.SetPercent(-45);
    Left_motor.SetPercent(-5);
    Sleep(1.1);
    Right_motor.Stop();
    Left_motor.Stop();

}

//function to turn a 45 degree angle to the right based on time - temporary
void TurnTimed45Right()
{
    Right_motor.SetPercent(5);
    Left_motor.SetPercent(45);
    Sleep(1.1);
    Right_motor.Stop();
    Left_motor.Stop();

}

//Function to count encoder value
void encoder_right(float a)
{
    int count = 0;
    while(count < a)
    {
    if(Right_encoder.Value() == 0)
    {
        count = count + 1;
        while(Right_encoder.Value() == 0)
        {}
    }
    if(Right_encoder.Value() == 1)
    {
        count = count + 1;
        while(Right_encoder.Value() == 1)
        {}
    }
    LCD.WriteLine(count);
    }
}

void check_x_minus(float x_coordinate) //using RPS while robot is in the -x direction
{
    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.X()>0) && (RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1))
    {
        if(RPS.X() > x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Forward(.25);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Backwards(.25);
        }
    }
}

void check_x_plus(float x_coordinate) //using RPS while robot is in the -x direction
{
    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.X()>0) && (RPS.X() < x_coordinate - 2 || RPS.X() > x_coordinate + 2))
    {
        if(RPS.X() > x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Backwards(.25);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Forward(.25);
        }
     Sleep(300);
    }
}

void check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
{
    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.Y()>0) && (RPS.Y() < y_coordinate - 0.5 || RPS.Y() > y_coordinate + 0.5))
    {
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Forward(.25);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Backwards(.25);
        }
    }
}

void check_y_plus(float y_coordinate) //using RPS while robot is in the +y direction
{
    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.Y()>0) && (RPS.Y() < y_coordinate - .25 || RPS.Y() > y_coordinate + .25))
    {
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Backwards(.25);
            Sleep(300);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            Forward(.25);
            Sleep(300);
        }
    }
}

void check_heading(float heading) //using RPS
{
    //you will need to fill out this one yourself and take into account
    //checking for proper RPS data and the edge conditions
    //(when you want the robot to go to 0 degrees or close to 0 degrees)
    while(RPS.Heading() >= 0 && (RPS.Heading()< heading - 2 || RPS.Heading() > heading + 2))
    {
        if(RPS.Heading() > heading)
        {
            Turn_Right(0,2);
            Sleep(300);
        }
        if(RPS.Heading() < heading)
        {
            Turn_Left(0,2);
            Sleep(300);
        }
    Sleep(300);
    }
}
/*
//function to test if left dig opt is reading values
 void LeftDigTest()
 {
    LCD.WriteLine("testing left optosensor");
    Sleep(1.0);
    LCD.Clear();
    while (true){
        LCD.WriteLine(Left_dig_opt.Value());
        Sleep(0.5);
    }
 }

//function to test is right dig opt is reading values
 void RightDigTest()
 {
    LCD.WriteLine("testing right optosensor");
    Sleep(1.0);
    LCD.Clear();
    while (true){
        LCD.WriteLine(Right_dig_opt.Value());
        Sleep(0.5);
    }
 }


//function to test if motors move straight forward
void TestMotors()
{
        Right_motor.SetPercent(-25);
        Left_motor.SetPercent(26);
        Sleep(5.0);
        Right_motor.Stop();
        Left_motor.Stop();
}


//function to test what values the digital optosensors are reading while the robot is moving
void TestEncoding()
{
    int left_true_count = 0, right_true_count = 0, left_false_count = 0, right_false_count = 0;

    Right_motor.SetPercent(-25);
    Left_motor.SetPercent(27);

    if (Right_dig_opt.Value() == 0){
        right_false_count++;
    }
    if (Left_dig_opt.Value() == 0){
        left_false_count++;
    }
    if (Right_dig_opt.Value() == 1){
        right_true_count++;
    }
    if (Left_dig_opt.Value() == 1){
        left_true_count++;
    }

    Sleep(3.0);

    Right_motor.Stop();
    Left_motor.Stop();



    LCD.WriteLine("Right false count: " + right_false_count);
    LCD.WriteLine("Left false count: " + left_false_count);
    LCD.WriteLine("Right true count: " + right_true_count);
    LCD.WriteLine("Left true count: " + left_true_count);
}



void TestCdS()
{
    while (true)
    {
     LCD.WriteLine(CdS_Cell.Value());
    }
}

void CdSControl()
{
    Tail.SetMin(TAIL_SERVO_MIN);
    Tail.SetMax(TAIL_SERVO_MAX);

    while (true)
    {
        if (CdS_Cell.Value() > 0 && CdS_Cell.Value() < 1.3)
        {
            Tail.SetDegree(90.0);
        }
        else if (CdS_Cell.Value() > 2.5 && CdS_Cell.Value() < 3.3)
        {
            Tail.SetDegree(180.0);
        }

    }

}



//function to calibrate a servo motor
void Calibration()
{
     Lever.TouchCalibrate();
}

void MoveTilCdS()
{
    Right_motor.SetPercent(-25);
    Left_motor.SetPercent(25);


    Right_motor.Stop();
    Left_motor.Stop();
}
*/

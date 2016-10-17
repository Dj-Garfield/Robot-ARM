///////////////////////////////////////////////////////
/* 
   Robot movement is based on the principle of inverse kinematics
   
   OPERATION
     Format:
     1. Any coordinate:
     {x,  y,  z}
     * -10 <= x <= 10
     * 1 <= y <= 10
     * 0 <= z <= 6

     2. Tiempo de espera:
       {0.001,  0.001,  T}
     * 0.001: reserved for a wait time.
     * "T" is the wait time, in milliseconds

     3. Open and close GRIPPER
     {0.002,  0.002,  G}
     * 0.002: reserved for open/close gripper.
     * 60 <= G <= 125 (G: degree sexagesimal)
      
     
  EXAMPLE:
  
  {-5,  0,  6.5},         // initial position
  {0.001,  0.001,  200},  // wait 200 milliseconds
  {-5,  0,  -1},          // nex position
  {0.001,  0.001,  200},  // wait 200 milliseconds
  {-8.5,  0,  -1},        // nex position
  {0.001,  0.001,  1000}, // wait 1000 milliseconds (1 second)
  {0.002,  0.002,  35},   // close gripper
  {0.001,  0.001,  1000}, // wait 1000 milliseconds (1 second)
  {-5,  0,  0},           // nex position
  {0.001,  0.001,  200},  // wait 200 milliseconds
  {-5,  0,  6.5},         // nex position
  {0.001,  0.001,  200},  // wait 200 milliseconds
  {5,  0,  6.5},          // nex position
  {0.001,  0.001,  200},  // wait 200 milliseconds
  {8.5,  0,  6.5},        // nex position
  {0.001,  0.001,  1000}, // wait 1000 milliseconds (1 second)
  {0.002,  0.002,  90},   // open gripper
  {0.001,  0.001,  2000}, // wait 2000 milliseconds (2 second)
  {5,  0,  6.5},          // nex position
  {0.001,  0.001,  200},  // wait 200 milliseconds
  {-5,  0,  6.5},         // initial position (It is going to the starting point to start a new cycle)

  AUTOR
   Fernandez Villanueva David
   fernandezv.david@gmail.com
   National University of Engineering LIMA-PERU
   Version 2.0

*/
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#include <Servo.h>
Servo miServo1; Servo miServo2; Servo miServo3; Servo miServo4;

void setup()
{
  Serial.begin(9600);
  miServo1.attach(5);  // AXIS Y
  miServo2.attach(6);  // AXIS Z
  miServo3.attach(9);  // GRIPPER
  miServo4.attach(10); // X
}

double X[] = {5, 0, -5};
double Y[] = {5, 5, 5};
double Z[] = {2, 2, 2};

//{00.00,  08.05,  02.00},
int speed_mov = 5;
const int num_xyz = 21;

///////////////////////////////////////////////////////////////////
double coordinates[num_xyz][3] = { 
//HEAR SHOULD WRITE THE COORDINATES
  {-5,  0,  6.5},
  {0.001,  0.001,  200},
  {-5,  0,  -1},
  {0.001,  0.001,  200},
  {-8.5,  0,  -1},
  {0.001,  0.001,  1000},
  {0.002,  0.002,  35},
  {0.001,  0.001,  1000},
  {-5,  0,  0},
  {0.001,  0.001,  200},
  {-5,  0,  6.5},
  {0.001,  0.001,  200},
  {5,  0,  6.5},
  {0.001,  0.001,  200},
  {8.5,  0,  6.5},
  {0.001,  0.001,  1000},
  {0.002,  0.002,  90},
  {0.001,  0.001,  2000},
  {5,  0,  6.5},
  {0.001,  0.001,  200},
  {-5,  0,  6.5},
  
 // END COORDINATES
};
/////////////////////////////////////////////////////////////////

const int mum_X = sizeof(X) / 4;

//double theta1[mum_X];
//double theta2[mum_X];
//double theta3[mum_X];

double theta1[num_xyz];
double theta2[num_xyz];
double theta3[num_xyz];

int alpha1, alpha2, alpha3, alpha4;
int vect_with_coord[num_xyz];



int c_theta1_2_3[num_xyz][4];

//int next_point_rate[11][6];
float next_point_rate[num_xyz][7];

const float pi = 3.14;

double L1 = 0;
double L2 = 6;
double L3 = 5.5;
double x, y, z;
double q11, q22, q33;
float h, c, alpha, ganma, q1, q2, q3;
int cont = 0;

double num_prueba = -10.9;
double num_abs;

int num_coord = 0;

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void loop()
{
  inv_kinematics();
  rate_add();
  delay(1000);
  while (1)
  {
    path_fin();
  }
}
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

double inv_kinematics()
{
  for (int i = 0; i < num_xyz; i++) // coordinates[0][0]
  {
    if ((coordinates[i][0] == coordinates[i][1]) && (coordinates[i][0] == 0.001))
    {
      c_theta1_2_3[i][0] = 1;
      c_theta1_2_3[i][1] = (int)(coordinates[i][2]);
      c_theta1_2_3[i][2] = (int)(coordinates[i][2]);
      c_theta1_2_3[i][3] = (int)(coordinates[i][2]);
    }
    else if ((coordinates[i][0] == coordinates[i][1]) && (coordinates[i][0] == 0.002))
    {
      c_theta1_2_3[i][0] = 2;
      c_theta1_2_3[i][1] = (int)(coordinates[i][2]);
      c_theta1_2_3[i][2] = (int)(coordinates[i][2]);
      c_theta1_2_3[i][3] = (int)(coordinates[i][2]);
    }
    else
    {
      x = coordinates[i][0];
      y = coordinates[i][1];
      z = coordinates[i][2];

      h = sqrt(pow(x, 2) + pow(y, 2));
      c = sqrt(pow(x, 2) + pow(y, 2) + pow((z - L1), 2));
      alpha = acos((pow(L2, 2) + pow(c, 2) - pow(L3, 2)) / (2 * L2 * c));
      ganma = atan((z - L1) / (x));

      q1 = atan(y / x) * 180 / pi;
      q2 = (atan((z - L1) / (h)) + alpha) * 180 / pi;
      q3 = (acos((pow(c, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3))) * 180 / pi;

      if (x < 0)
      {
        q11 = 180 + (int)(q1); //(int)(q1);
      }
      else
      {
        q11 = (int)(q1); //(int)(q1);
      }
      q22 = (int)(q2); //(int)(q2);
      q33 = (int)(q3); //(int)(q3);

      c_theta1_2_3[i][0] = 0;
      c_theta1_2_3[i][1] = q11;
      c_theta1_2_3[i][2] = q22;
      c_theta1_2_3[i][3] = q33;


      theta1[i] = q11;
      theta2[i] = q22;
      theta3[i] = q33;
      cont = cont + 1;
    }
  }
  //comprueva_array2();

}


double absoluto(double num1)
{
  if (num1 < 0)
  {
    return (-1) * num1;
  }
  else
  {
    return num1;
  }

}



void rate_add()
{ //int next_point_rate[11][6];
  int cont_rate_add = 0;
  int first_coord = 0;

  //int vect_with_coord[11];
  int num_no_coord = 0;
  //int num_coord=0;
  for (int j = 0; j < num_xyz; j++)
  {
    if (c_theta1_2_3[j][0] == 0)
    {
      vect_with_coord[num_coord] = j;
      num_coord = 1 + num_coord; // número de coordenadas "0".
    }
    else
    {
      //vect_with_coord[10 - num_no_coord] = 99;
      vect_with_coord[(num_xyz-1) - num_no_coord] = 99;
      num_no_coord = num_no_coord + 1;
    }
  }

  // rate calculate
  int cont_num_coord = 0;
  for (int i = 0; i < num_xyz; i++)
  {
    if (c_theta1_2_3[i][0] == 0)
    {
      if (i == vect_with_coord[num_coord - 1])
      {
        int k0 = vect_with_coord[cont_num_coord];
        int k1 = vect_with_coord[0];
        next_point_rate[i][0] = c_theta1_2_3[k1][1] - c_theta1_2_3[k0][1];
        next_point_rate[i][1] = c_theta1_2_3[k1][2] - c_theta1_2_3[k0][2];
        next_point_rate[i][2] = c_theta1_2_3[k1][3] - c_theta1_2_3[k0][3];

        int maxim_tree_val = max_tree(next_point_rate[i][0], next_point_rate[i][1], next_point_rate[i][2]);

        next_point_rate[i][3] = next_point_rate[i][0] / maxim_tree_val;
        next_point_rate[i][4] = next_point_rate[i][1] / maxim_tree_val;
        next_point_rate[i][5] = next_point_rate[i][2] / maxim_tree_val;
        next_point_rate[i][6] = maxim_tree_val;

        cont_num_coord = cont_num_coord + 1;
      }
      else
      {
        int k0 = vect_with_coord[cont_num_coord];
        int k1 = vect_with_coord[cont_num_coord + 1];
        next_point_rate[i][0] = c_theta1_2_3[k1][1] - c_theta1_2_3[k0][1];
        next_point_rate[i][1] = c_theta1_2_3[k1][2] - c_theta1_2_3[k0][2];
        next_point_rate[i][2] = c_theta1_2_3[k1][3] - c_theta1_2_3[k0][3];

        int maxim_tree_val = max_tree(next_point_rate[i][0], next_point_rate[i][1], next_point_rate[i][2]);

        next_point_rate[i][3] = next_point_rate[i][0] / maxim_tree_val;
        next_point_rate[i][4] = next_point_rate[i][1] / maxim_tree_val;
        next_point_rate[i][5] = next_point_rate[i][2] / maxim_tree_val;
        next_point_rate[i][6] = maxim_tree_val;

        cont_num_coord = cont_num_coord + 1;
      }
    }
  }
}







int max_tree(int num1, int num2, int num3)
{
  if (num1 < 0) {
    num1 = num1 * (-1);
  }
  if (num2 < 0) {
    num2 = num2 * (-1);
  }
  if (num3 < 0) {
    num3 = num3 * (-1);
  }

  int borra = max(num1, num2);
  int maxim_tree = max(borra, num3);

  return maxim_tree;
}


void path_fin()
{
  int inicio = 0;
  for (int i = 0; i < num_xyz; i++)
  { ///////////


    if (c_theta1_2_3[i][0] == 0)
    {
      if (inicio == 0) //first loop
      {
        delay(speed_mov);
        alpha1 = (int)(c_theta1_2_3[i][1]);
        alpha2 = (int)(c_theta1_2_3[i][2]);
        alpha3 = (int)(c_theta1_2_3[i][3]);
        miServo4.write(43+alpha2);             
        miServo1.write(55+(alpha3-alpha2));
        miServo2.write(alpha1);              

      }
      else
      {
        int netxc = vect_with_coord[inicio - 1];
        for (int j = 1; j <= next_point_rate[netxc][6]; j++)
        {
          delay(speed_mov);
          alpha1 = (int)(c_theta1_2_3[netxc][1] + j * next_point_rate[netxc][3]);
          alpha2 = (int)(c_theta1_2_3[netxc][2] + j * next_point_rate[netxc][4]);
          alpha3 = (int)(c_theta1_2_3[netxc][3] + j * next_point_rate[netxc][5]);
          miServo4.write(43+alpha2);              
          miServo1.write(55+(alpha3-alpha2));  
          miServo2.write(alpha1);                 
        }
      }
      inicio = inicio + 1;
    }

    else if (c_theta1_2_3[i][0] == 1)// retardo
    {
      delay(c_theta1_2_3[i][2]);
      //Serial.println(c_theta1_2_3[i][2]);
      //delay(500);
    }
    else                            
    {
      int gripper_g = (int)(c_theta1_2_3[i][2]);
      miServo3.write(gripper_g);
      //int miServo3();
    }

    //
  }// end for




}




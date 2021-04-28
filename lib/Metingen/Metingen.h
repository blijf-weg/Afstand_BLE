#include <math.h>
#include <Arduino.h>
class Metingen{
    public: 
        double x0 = 0;
        double y0 = 0;

        double x1 = 3.7;
        double y1 = 0;

        double x2 = 0;
        double y2 = 2.95;
        double rssi_waarden[6];
        double afstanden[3];

        Metingen(){
            for(int i = 0; i<6; i++){
                rssi_waarden[i]=0;
            }
            for(int i = 0; i<3 ; i++){
                afstanden[i] = 0;
            }
        }
        void addRSSI(double RSSI, int punt){
            rssi_waarden[punt] = RSSI;
            if(punt < 3)
                rssi_waarden[punt + 3] = 1;
        }
        void addAfstand(double afstand, int punt){
            afstanden[punt] = afstand;
        }
        double* berekenPositie(){
            bool a = true;
            for(int i = 3; i < 6; i++){
                if(rssi_waarden[i] == 0)
                    a = false;
            }
            if(a){
                for(int i = 0; i < 3; i++){
                    Serial.print(rssi_waarden[i]);
                    Serial.print(" ,");
                    Serial.println(afstanden[i], 6);
                    rssi_waarden[i + 3] = 0;
                }
                double* returnWaarde;
                returnWaarde =  berekenSnijpunten(); 
                return returnWaarde;
            }
            else{
                return nullptr;
            }
        }
        double* berekenSnijpunten(){
            //Snijpunten tss cirkel 0 en cirkel 1
            double x01[2]; 
            double y01[2];
            berekenSnijpunt(x0,y0, afstanden[0], x1,y1, afstanden[1],&x01[0],&y01[0],&x01[1],&y01[1]);
            //Snijpunten tss cirkel 0 en cirkle 2
            double x02[2]; 
            double y02[2];
            berekenSnijpunt(x0,y0, afstanden[0], x2,y2, afstanden[2],&x02[0],&y02[0],&x02[1],&y02[1]);
            //Snijpunten tss cirkel 1 en cirkel 2
            double x12[2]; 
            double y12[2];
            berekenSnijpunt(x1,y1, afstanden[1], x2,y2, afstanden[2],&x12[0],&y12[0],&x12[1],&y12[1]);

            /*
            Serial.println("//Snijpunten tss cirkel 0 en cirkel 1");
            Serial.print(x01[0],4);
            Serial.print(", ");
            Serial.println(y01[0],4);
            Serial.print(x01[1],4);
            Serial.print(", ");
            Serial.println(y01[1],4);

            Serial.println("//Snijpunten tss cirkel 0 en cirkel 2");
            Serial.print(x02[0],4);
            Serial.print(", ");
            Serial.println(y02[0],4);
            Serial.print(x02[1],4);
            Serial.print(", ");
            Serial.println(y02[1],4);

            Serial.println("//Snijpunten tss cirkel 1 en cirkel 2");
            Serial.print(x12[0],4);
            Serial.print(", ");
            Serial.println(y12[0],4);
            Serial.print(x12[1],4);
            Serial.print(", ");
            Serial.println(y12[1],4);
            */

            //Keuze van de juiste snijpunten
            double x_punt0 = 0;
            double x_punt1 = 0;
            double x_punt2 = 0;

            double y_punt0 = 0;
            double y_punt1 = 0;
            double y_punt2 = 0;


            //De juiste snijpunten selecteren => De drie snijpunten die het dichts bij elkaar liggen 
            //Omtrek van de driehoek berekenen voor punten die het dichts bij elkaar liggen
            double kleinsteOmtrek = 2000000;
            int u = 0;
            int v = 0;
            int w = 0;
            for(int i = 0; i < 2; i++){
                //if(x01[i] > 0 && y01[i] > 0){
                    for(int j = 0; j<2; j++){
                        //if(x02[j] > 0 && y02[j] > 0){
                            for(int k = 0;k <2; k++){
                                //if(x12[k] > 0 && y12[k] > 0){
                                    double afstand01_02 = pow((x01[i] - x02[j]),2) + pow((y01[i] - y02[j]),2);  
                                    double afstand01_12 = pow((x01[i] - x12[k]),2) +  pow((y01[i] - y12[k]),2) ;
                                    double afstand02_12 = pow((x02[j] - x12[k]),2)  + pow((y02[j] - y12[k]),2);
                                    double omtrek = afstand01_02 + afstand01_12 + afstand02_12;
                                    if(kleinsteOmtrek > omtrek){
                                        kleinsteOmtrek = omtrek;
                                        u = i;
                                        v = j;
                                        w = k; 
                                    }
                                //}
                            }
                        //}
                    }
                //}

            }

            
            Serial.println("Juiste snijpunten ");

            x_punt0 = x01[u];
            y_punt0 = y01[u];
            Serial.print( x_punt0,4);
            Serial.print(", ");
            Serial.println( y_punt0,4);


            x_punt1 = x02[v];
            y_punt1 = y02[v];
            Serial.print( x_punt1,4);
            Serial.print(", ");
            Serial.println( y_punt1,4);

            x_punt2 = x12[w];
            y_punt2 = y12[w];
            Serial.print( x_punt2,4);
            Serial.print(", ");
            Serial.println( y_punt2,4);
            

        
            double x_punt = (x_punt0 + x_punt1 + x_punt2) /3; 
            double y_punt = (y_punt0 + y_punt1 + y_punt2) /3; 

            Serial.println("Punt: ");
            Serial.print(x_punt,4);
            Serial.print(", ");
            Serial.println(y_punt,4);
            
            static double array[2];
            array[0] = x_punt;
            array[1] = y_punt; 

            return array;
        }
        int berekenSnijpunt(double x0, double y0, double r0,double x1, double y1, double r1,double *xi, double *yi,double *xi_prime, double *yi_prime){
            double a, dx, dy, d, h, rx, ry;
            double x2, y2;

            /* dx and dy are the vertical and horizontal distances between
            * the circle centers.
            */
            dx = x1 - x0;
            dy = y1 - y0;

            /* Determine the straight-line distance between the centers. */
            //d = sqrt((dy*dy) + (dx*dx));
            d = hypot(dx,dy); // Suggested by Keith Briggs

            /* Check for solvability. */
            if (d > (r0 + r1))
            {
                /* no solution. circles do not intersect. */
                return 2000000;
            }
            if (d < fabs(r0 - r1))
            {
                /* no solution. one circle is contained in the other */
                return 2000000;
            }

            /* 'point 2' is the point where the line through the circle
            * intersection points crosses the line between the circle
            * centers.
            */

            /* Determine the distance from point 0 to point 2. */
            a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

            /* Determine the coordinates of point 2. */
            x2 = x0 + (dx * a/d);
            y2 = y0 + (dy * a/d);

            /* Determine the distance from point 2 to either of the
            * intersection points.
            */
            h = sqrt((r0*r0) - (a*a));

            /* Now determine the offsets of the intersection points from
            * point 2.
            */
            rx = -dy * (h/d);
            ry = dx * (h/d);

            /* Determine the absolute intersection points. */
            *xi = x2 + rx;
            *xi_prime = x2 - rx;
            *yi = y2 + ry;
            *yi_prime = y2 - ry;

            return 1;
        }
};
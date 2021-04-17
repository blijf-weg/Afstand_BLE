#include <math.h>
#include <Arduino.h>
class Metingen{
    public: 
        double x0 = 0;
        double y0 = 0;

        double x1 = 2;
        double y1 = 0;

        double x2 = 0;
        double y2 = 1.1;
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
            rssi_waarden[punt + 3] = 1;
            berekenAfstanden();
        }
        void berekenAfstanden(){
            bool a = true;
            for(int i = 3; i < 6; i++){
                if(rssi_waarden[i] == 0)
                    a = false;
            }
            if(a){
                for(int i = 0 ; i< 3; i++){
                    if(rssi_waarden[i] > -35){
                        afstanden[i] = pow(10, (-28 - rssi_waarden[i])/(10*3));
                    }
                    else if (rssi_waarden[i] > -48)
                    {
                       afstanden[i] = pow(10, (-41 - rssi_waarden[i])/(10*3));
                    }
                    else 
                    {
                        afstanden[i] =pow(10, (- 40 - rssi_waarden[i])/(10*3));
                    }
                    rssi_waarden[i+3] = 0;   
                }
                for(int i = 0; i < 3; i++){
                    Serial.print(rssi_waarden[i]);
                    Serial.print(" ,");
                    Serial.println(afstanden[i], 6);
                }
                berekenSnijpunten();
            }
        }
        void berekenSnijpunten(){
            //Snijpunten tss cirkel 0 en cirkel 1
            double x01_1;
            double y01_1;
            double x01_2;
            double y01_2;
            berekenSnijpunt(x0,y0, afstanden[0], x1,y1, afstanden[1],&x01_1,&y01_1,&x01_2,&y01_2);
            //Snijpunten tss cirkel 0 en cirkle 2
            double x02_1;
            double y02_1;
            double x02_2;
            double y02_2;
            berekenSnijpunt(x0,y0, afstanden[0], x2,y2, afstanden[2],&x02_1,&y02_1,&x02_2,&y02_2);
            //Snijpunten tss cirkel 1 en cirkel 2
            double x12_1;
            double y12_1;
            double x12_2;
            double y12_2;
            berekenSnijpunt(x1,y1, afstanden[1], x2,y2, afstanden[2],&x12_1,&y12_1,&x12_2,&y12_2);

            Serial.println("//Snijpunten tss cirkel 0 en cirkel 1");
            Serial.print(x01_1,4);
            Serial.print(", ");
            Serial.println(y01_1,4);
            Serial.print(x01_2,4);
            Serial.print(", ");
            Serial.println(y01_2,4);

            Serial.println("//Snijpunten tss cirkel 0 en cirkel 2");
            Serial.print(x02_1,4);
            Serial.print(", ");
            Serial.println(y02_1,4);
            Serial.print(x02_2,4);
            Serial.print(", ");
            Serial.println(y02_2,4);

            Serial.println("//Snijpunten tss cirkel 1 en cirkel 2");
            Serial.print(x12_1,4);
            Serial.print(", ");
            Serial.println(y12_1,4);
            Serial.print(x12_2,4);
            Serial.print(", ");
            Serial.println(y12_2,4);

            //Keuze van de juiste snijpunten
            double x_punt0 = 0;
            double x_punt1 = 0;
            double x_punt2 = 0;

            double y_punt0 = 0;
            double y_punt1 = 0;
            double y_punt2 = 0;

            if(x01_1 > 0 && y01_1 > 0){
                 x_punt0 = x01_1;
                 y_punt0 = y01_1;
            }else{
                 x_punt0 = x01_2;
                 y_punt0 = y01_2; 
            }

            if(x02_1 > 0 && y02_1 > 0){
                 x_punt1 = x02_1;
                 y_punt1 = y02_1;
            }else{
                 x_punt1 = x02_2;
                 y_punt1 = y02_2; 
            }

            if(x12_1 > 0 && y12_1 > 0){
                 x_punt2 = x12_1;
                 y_punt2 = y12_1;
            }else{
                 x_punt2 = x12_2;
                 y_punt2 = y12_2; 
            }

            double x_punt = (x_punt0 + x_punt1 + x_punt2) /3; 
            double y_punt = (y_punt0 + y_punt1 + y_punt2) /3; 
            Serial.println(x_punt,4);
            Serial.println(y_punt,4);
            delay(5000);
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
                return 0;
            }
            if (d < fabs(r0 - r1))
            {
                /* no solution. one circle is contained in the other */
                return 0;
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
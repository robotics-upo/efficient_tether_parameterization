#include <cmath>
#include <random> 

// Ceres libraries
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <iostream>
#include <fstream>

// Workspace
using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::LossFunction;
using ceres::AutoDiffCostFunction;


using namespace std;

class Point {
public:
    double x, y;

    Point(double x_val = 0, double y_val = 0) : x(x_val), y(y_val) {}
};

class WriteData{
    public:
        std::ifstream ifile;
		std::ofstream ofs;
        std::string file_;

        WriteData(string name_){

            const char* home = getenv("HOME"); // Extender a ruta explicita de home
            if (home != nullptr) {
                file_ = std::string(home) + "/results_error_catenary_vs_parabola_"+name_+".txt";
            } else {
                file_ = "results_error_catenary_vs_parabola_"+name_+".txt";  // Ruta de fallback
            }
            openFile();
        }

        ~WriteData(void)
        {
            closeFile();
        }

        void openFile(){
            ifile.open(file_);
            if(ifile) {
                std::cout << "      " << file_ <<" : File exists !!!!!!!!!! " << std::endl;
            } else {
                std::cout << "      " << file_ <<" : File doesn't exist !!!!!!!!!! " << std::endl;
            }
            
            ofs.open(file_.c_str(), std::ofstream::app);
        }

        void writeInFile(double l_, double x_, double y_cat_, double y_par_, double error_){

            if (ofs.is_open()) {
                ofs << l_ << ","
                    << x_ << "," 
                    << y_cat_ << "," 
                    << y_par_ << "," 
                    << error_ << ";";
            } 
            else 
                std::cout << "Couldn't open " << file_ << " for writing." << std::endl;
        }   

        void closeFile(){
            if (ofs.is_open()) {
                ofs << std::endl;
                ofs.close(); 
            } 
            else 
                std::cout << "      Closed " << file_ << " file !!!!!." << std::endl;
        }
};

class CatenaryParameters
{
    public:
        CatenaryParameters(double xA_, double yA_, double xB_, double yB_, float l_)
        {
            // Hunging point pA(xA,yA) and pB(xB,yB)
            xA = xA_;
            yA = yA_;
            xB = xB_; 
            yB = yB_; 
            L = l_;
        }

        ~CatenaryParameters(void) 
        {}

        template <typename T>
        bool operator()(const T* P_, T* R_) const 
        {   
            R_[0] = T{100.0}*(P_[2] * cosh((xA - P_[0])/P_[2]) + P_[1]  - yA);
            R_[1] = T{100.0}*(P_[2] * cosh((xB - P_[0])/P_[2]) + P_[1]  - yB);
            T La = P_[2]*tanh((xA - P_[0])/P_[2])*sqrt(sinh((xA - P_[0])/P_[2])*sinh((xA - P_[0])/P_[2]) + T{1.0});
            T Lb = P_[2]*tanh((xB - P_[0])/P_[2])*sqrt(sinh((xB - P_[0])/P_[2])*sinh((xB - P_[0])/P_[2]) + T{1.0});
            T len = Lb - La;
            R_[2] = (len - L);
            // std::cout << "          Length : L = " << L << " , len = " << len << std::endl;   
            return true;
        }

    private:
        // Point to be evaluated
        double xA, yA, xB, yB, L;
};

class ParabolaParameters
{
    public:
        ParabolaParameters(double xA_, double yA_, double xB_, double yB_, float l_)
        {
            // Hunging point pA(xA,yA) and pB(xB,yB)
            xA = xA_;
            yA = yA_;
            xB = xB_; 
            yB = yB_; 
            L = l_;
        }

        ~ParabolaParameters(void) 
        {}

        template <typename T>
        bool operator()(const T* P_, T* R_) const 
        {
            T val, La, Lb;
            R_[0] = T{100.0}*(P_[0] * xA * xA + P_[1]*xA + P_[2] - yA);
            R_[1] = T{100.0}*(P_[0] * xB * xB + P_[1]*xB + P_[2] - yB);
            val = sqrt((T{2.0}*P_[0]*xA) * (T{2.0}*P_[0]*xA) + (T{4.0}*P_[0]*P_[1]*xA) + (P_[1]*P_[1] ) + T{1.0}); // Evaluate in Xa
            La = log(val + (T{2.0}*xA*P_[1]*P_[1] + P_[0]*P_[1])/(P_[0]))/(T{4.0}*(P_[0])) + (xA/T{2.0} + P_[1]/(T{4.0}*(P_[0])))*val;
            val = sqrt((T{2.0}*P_[0]*xB) * (T{2.0}*P_[0]*xB) + (T{4.0}*P_[0]*P_[1]*xB) + (P_[1]*P_[1] ) + T{1.0}); // Evaluate in Xb
            Lb = log(val + (T{2.0}*xB*P_[1]*P_[1] + P_[0]*P_[1])/(P_[0]))/(T{4.0}*(P_[0])) + (xB/T{2.0} + P_[1]/(T{4.0}*(P_[0])))*val;

			T len = Lb - La;
            R_[2] = T{20.0}*(len - L);
            // std::cout << "      Length : L = " << L << " , len = " << len << std::endl;   
            // std::cout << "      P : [0] = " << P_[0] << " , [1] = " << P_[1] << " , [2] = " << P_[2] << std::endl;   
            std::cout << "      R : [0] = " << R_[0] << " , [1] = " << R_[1] << " , [2] = " << R_[2] << std::endl;   
            return true;
        }

    private:
        // Point to be evaluated
        double xA, yA, xB, yB, L;
};


class FindParameters{
    public:
        double p, q, r;
        double param_x0, param_y0, param_a, param_p, param_q, param_r;
        int max_num_iterations = 5000;
        int count1 = 1;
        int count2 = 1;

        FindParameters()
        {}

        ~FindParameters(void)
        {} 

        void initialSolutions(double p1_, double p2_, double p3_ , double c1_, double c2_, double c3_){
            param_x0 = p1_;  
            param_y0 = p2_;  
            param_a = p3_;
            param_p = c1_;  
            param_q = c2_;  
            param_r = c3_;
        }

        bool solve(double _xA, double _yA, double _xB, double _yB, float _l, int mode_ = 1)
        {
            // mode = 1 -> optimization Parabola && Catenary      mode = 2 -> optimization Catenary      mode = 3 -> optimization Parabola

            double cat[3], par[3];
            /******************** Initial Catenary solution **********************/ 

            if (mode_ == 1 || mode_ == 2){
                cat[0] = param_x0; // x0 
                cat[1] = param_y0; // y0
                cat[2] = param_a; // a      
                // Build the problem.
                Problem problem1;
                // Set up a cost funtion per point into the cloud
                CostFunction* cf1 = new ceres::AutoDiffCostFunction<CatenaryParameters, 3, 3>( new CatenaryParameters(_xA, _yA, _xB, _yB, _l) );
                problem1.AddResidualBlock(cf1, NULL, cat);
                problem1.SetParameterLowerBound(cat, 0, 0.0);
                problem1.SetParameterLowerBound(cat, 2, 0.1);
                // Run the solver!
                Solver::Options options;
                options.minimizer_progress_to_stdout = false;
                options.max_num_iterations = max_num_iterations;
                Solver::Summary summary1;
                if (mode_ == 2){
                    std::cout << "     ["<< count1 <<"]INITIALIZING OPTIMIZATION FOR CATENARY PARAMETERS" << std::endl;
                    count1 ++;
                }else
                    std::cout << "  ** INITIALIZING OPTIMIZATION FOR CATENARY PARAMETERS" << std::endl;
                Solve(options, &problem1, &summary1);

                double La = cat[2]*tanh((_xA - cat[0])/cat[2])*sqrt(sinh((_xA - cat[0])/cat[2])*sinh((_xA - cat[0])/cat[2]) + 1.0);
                double Lb = cat[2]*tanh((_xB - cat[0])/cat[2])*sqrt(sinh((_xB - cat[0])/cat[2])*sinh((_xB - cat[0])/cat[2]) + 1.0);
                double Lfinal = Lb - La;
                std::cout <<"          Lfinal = " << Lfinal << " , Lesperado = "<< _l << std::endl;
                double Xm = (_xA + _xB)/2.0;
                double Y_ = cat[2] * cosh((Xm - cat[0])/cat[2]) + ( cat[1]);

                std::cout <<"          Y = " << Y_ << std::endl;
                // if(summary1.message == "Initial residual and Jacobian evaluation failed.")
                //     printf("\t\t <<<< Failed in status");
                // Some debug information
                // std::cout << summary1.BriefReport() << "\n";
            }

            /******************** Initial Parabola solution **********************/ 

            if (mode_ == 1 || mode_ == 3){
                par[0] = param_p; // p 
                par[1] = param_q; // q
                par[2] = param_r; // r 
                // Build the problem.
                Problem problem2;
                // Set up a cost funtion per point into the cloud
                CostFunction* cf2 = new ceres::AutoDiffCostFunction<ParabolaParameters, 3, 3>( new ParabolaParameters(_xA, _yA, _xB, _yB, _l) );
                problem2.AddResidualBlock(cf2, NULL, par);
                problem2.SetParameterLowerBound(par, 0, 0.01);
                problem2.SetParameterLowerBound(par, 2, _yA);
                Solver::Summary summary2;
                if (mode_ == 3){
                    std::cout << "     ["<< count2 <<"]INITIALIZING OPTIMIZATION FOR PARABOLA PARAMETERS" << std::endl;
                    count2 ++;
                }else
                    std::cout << "  ** INITIALIZING OPTIMIZATION FOR PARABOLA PARAMETERS" << std::endl;
                // Run the solver!
                Solver::Options options2;
                options2.minimizer_progress_to_stdout = false;
                options2.max_num_iterations = max_num_iterations;
                Solve(options2, &problem2, &summary2);

                double val = sqrt((2.0*par[0]*_xA) * (2.0*par[0]*_xA) + (4.0*par[0]*par[1]*_xA) + (par[1]*par[1] ) + 1.0);
                double La = log(val + (2.0*_xA*par[1]*par[1] + par[0]*par[1])/(par[0]))/(4.0*(par[0])) + (_xA/2.0 + par[1]/(4.0*(par[0])))*val;
                val = sqrt((2.0*par[0]*_xB) * (2.0*par[0]*_xB) + (4.0*par[0]*par[1]*_xB) + (par[1]*par[1] ) + 1.0);
                double Lb = log(val + (2.0*_xB*par[1]*par[1] + par[0]*par[1])/(par[0]))/(4.0*(par[0])) + (_xB/2.0 + par[1]/(4.0*(par[0])))*val;
                double Lfinal = (Lb - La) ;
                std::cout << "          Params Par: [0] = " << par[0]  << " , [1] = " << par[1] << " , [2] = " << par[2] << std::endl; 
                std::cout << "          Lfinal = " << Lfinal << " , Lesperado = "<< _l << std::endl;
                double Xm = (_xA + _xB)/2.0;
                double Y_ =  par[0] * Xm * Xm + par[1] * Xm + par[2];

                std::cout <<"          Y = " << Y_ << std::endl;
                // if(summary2.message == "Initial residual and Jacobian evaluation failed.")
                //     printf("\t\t <<<< Failed in status");
                // Some debug information
                // std::cout << summary2.BriefReport() << "\n";
            }
            param_x0 = cat[0];  param_y0 = cat[1];  param_a  = cat[2];
            param_p  = par[0];  param_q  = par[1];  param_r  = par[2];
                
            return true; 
        }        
};

class ComputePointsUsingPamameters{
    public:
        int num_sample_ = 20; // 20 is the number of examples per meter
        ComputePointsUsingPamameters(const Point& A_, const Point& B_, double L_, double c1_, double c2_, double c3_, double p1_, double p2_, double p3_, string n_){
            int samples_ = int(L_) * num_sample_;
            if (L_ < 1.0)
                samples_ = num_sample_;
            double step = fabs(B_.x - A_.x) / double(samples_);
            Point cat_, par_;

            WriteData wd(n_);

            for(int i=0 ; i < samples_+1; i++){
                cat_.x = par_.x = step * i;
                cat_.y = c3_ * cosh((cat_.x - c1_)/c3_) + ( c2_);
                par_.y = p1_ * par_.x * par_.x + p2_ * par_.x + p3_;
                // double error = fabs( cat_.y - par_.y);
                double error = ( cat_.y - par_.y);
                wd.writeInFile(L_, cat_.x, cat_.y, par_.y, error);
            }
            wd.closeFile();
        }
        ~ComputePointsUsingPamameters()
        {}

    private:
};

class ByLength {
    public:
        double param_x0, param_y0, param_a, param_p, param_q, param_r;
        // Constructor by Length 
        ByLength(const Point& A_, const Point& B_, double L_, double D_) {
            FindParameters fp;
            fp.initialSolutions(0.5, -1.0 , 1.5, 0.3, -0.5, A_.y);
            fp.solve(A_.x, A_.y, B_.x, B_.y, L_, 1);
            param_x0 = fp.param_x0; param_y0 = fp.param_y0; param_a = fp.param_a; 
            param_p = fp.param_p ; param_q = fp.param_q ; param_r = fp.param_r ;
        }
};

class ByFitting {
    public:
        double param_x0, param_y0, param_a, param_p, param_q, param_r;
        Point max_d_pt;

        // Constructor by fitting 
        ByFitting(const Point& A_, const Point& B_, double L_, double D_) {

            FindParameters fp;
            fp.initialSolutions(0.5, -1.0 , 1.5, 0.3, -0.5, A_.y);
            fp.solve(A_.x, A_.y, B_.x, B_.y, L_, 3); // solve() in mode = 3: only compute Parabola

            double error_ = 0.01;
            double L0, L1, Lm;
            L0 = D_;
            L1 = 30.0;
            int count = 0;
            while (true) {
                if (count == 0)
                    Lm = L_;
                else
                    Lm = (L0 + L1)/2.0;
                fp.solve(A_.x, A_.y, B_.x, B_.y, Lm, 2); // solve() in  mode = 2: only compute Catenary

                if (fabs(L1-L0)<= error_){
                    break;
                }
                double d_ = getMaxDistanceAxis(A_, B_, Lm, fp.param_x0, fp.param_y0, fp.param_a, fp.param_p, fp.param_q, fp.param_r);
                if (d_ < 0.0 )
                    L0 = Lm;
                else
                    L1 = Lm;
                count ++;
            }
            param_x0 = fp.param_x0; param_y0 = fp.param_y0; param_a = fp.param_a; 
            param_p = fp.param_p ; param_q = fp.param_q ; param_r = fp.param_r ;
        }

        double getMaxDistanceAxis(const Point& A_, const Point& B_, double L_, double c1_, double c2_, double c3_, double p1_, double p2_, double p3_){
            int num_sample_ = 20; // 20 is the number of examples per meter
            int samples_ = int(L_) * num_sample_;
            if (L_ < 1.0)
                samples_ = num_sample_;
            double step = fabs(B_.x - A_.x) / double(samples_);
            Point cat_, par_;

            double max_d_error = 0.0;
            for(int i=0 ; i < samples_+1; i++){
                cat_.x = par_.x = step * i;
                cat_.y = c3_ * cosh((cat_.x - c1_)/c3_) + ( c2_);
                par_.y = p1_ * par_.x * par_.x + p2_ * par_.x + p3_;
                if (fabs( par_.y - cat_.y) > fabs(max_d_error)){
                    max_d_error =  par_.y - cat_.y;
                    max_d_pt.x = cat_.x;
                    max_d_pt.y = cat_.y;
                }
            }
            return max_d_error;
        }
};


int main(int argc, char* argv[]) {

    double x1, y1, x2, y2, D, random_number;
    x1 = 0; y1 = 1;
    x2 = 12 ; y2 = 2;
    int count = 1;
    std::cout << std::endl << "**** Ready for computing." << std::endl;
    
    for(int i = 0; i < 1; i ++){
        for(int j = 0 ; j < 4 ; j++){ //Counting number of different UAV position in Y axe
            Point A(x1, y1);
            Point B(x2, y2);
            D = sqrt(pow(A.x-B.x,2)+ pow(A.y-B.y,2));
            std::cout << std::endl << "["<< count <<"]Points: A["<< A.x << ","<< A.y<<"] B["<< B.x << ","<< B.y<<"] ." << std::endl;

            // Generate Random Number for Length
            std::random_device rd;   // Random Origen
            std::mt19937 gen(rd());  // Mersenne Twister: generador de números pseudoaleatorios
            // Distribution number between D and 2.0*D (max_L : 15 or 20 or 30 mts)
            double length_max = 30.0;
            double min_L = D;
            double max_L = 2.0*D;
            if (max_L > length_max)
                max_L = length_max;
            std::uniform_real_distribution<> distrib(min_L, max_L);

            double aux_ = 0.0;
            for(int k=0; k < 1 ; k++){ // Count number of different tether Length
                do{
                    random_number = distrib(gen);
                    random_number = round(random_number * 10.0) / 10.0;

                }while(random_number == aux_);        

                aux_ = random_number;
                std::cout <<  "L=["<< random_number <<"] , D=["<< D <<"]" << std::endl;
                
                std::cout <<  " USING byLength Method" << std::endl;
                ByLength bL(A,B,random_number,D); // (Point A, Point B, Random L)
                ComputePointsUsingPamameters cpbl(A, B, random_number, bL.param_x0, bL.param_y0, bL.param_a, bL.param_p, bL.param_q, bL.param_r,"byLength");
                
                std::cout <<  " USING byFitting Method" << std::endl;
                ByFitting bF(A,B,random_number,D);
                ComputePointsUsingPamameters cpbf(A, B, random_number, bF.param_x0, bF.param_y0, bF.param_a, bF.param_p, bF.param_q, bF.param_r,"byFitting");
            }
            y2 = y2 + 2.0;
            count++;
        }
        y2 = 2.0;
        // x2 = x2 + 2*(1+i);
        x2 = x2 + (2+i);
    }
    
    return 0;
}
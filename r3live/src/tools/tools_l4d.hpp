#include <Eigen/Eigen>
namespace Common_tools
{
    
    inline void normalize_if_large(Eigen::Matrix<double,3,1> && mat, double val)
    {
        if (mat.norm()>val)
        {
            mat = mat.normalized()*val;
        }
    }
}
#ifndef POINTCLOUDNORMALHANDLER_H
#define POINTCLOUDNORMALHANDLER_H

#include <pcl/visualization/point_cloud_color_handlers.h>

namespace pcl
{
namespace visualization
{

template <class PointT>
class PointCloudNormalHandler : public PointCloudColorHandler<PointT>
{
    typedef PointCloudColorHandler<PointT>::PointCloud PointCloud;
//    typedef PointCloud::Ptr PointCloudPtr;
//    typedef PointCloud::ConstPtr PointCloudConstPtr;
public:
    typedef boost::shared_ptr<PointCloudNormalHandler<PointT>> Ptr;
    typedef boost::shared_ptr<const PointCloudNormalHandler<PointT>> ConstPtr;

    /** \brief Constructor. */
    PointCloudNormalHandler ()
    {
      capable_ = false;
    }

    /** \brief Constructor. */
    PointCloudNormalHandler (const PointCloudConstPtr &cloud, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals)
      : PointCloudColorHandler<PointT> (cloud), normals_(normals)
    {
        capable_ = true;
        setInputCloud (cloud);
    }

    /** \brief Destructor. */
    virtual ~PointCloudNormalHandler () {}

    /** \brief Get the name of the field used. */
    virtual std::string
    getFieldName () const { return ("normal"); }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
      * \param[out] scalars the output scalars containing the color for the dataset
      * \return true if the operation was successful (the handler is capable and
      * the input cloud was given as a valid pointer), false otherwise
      */
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (3);

        vtkIdType nr_points = normals_->points.size ();
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

        // Get a random color
        unsigned char* colors = new unsigned char[nr_points * 3];

        // Color every point
        for (vtkIdType cp = 0; cp < nr_points; ++cp)
        {
            const pcl::Normal &normal = normals_->points[cp];

            if (!pcl_isfinite (normal.normal_x) ||
                    !pcl_isfinite (normal.normal_y) ||
                    !pcl_isfinite (normal.normal_z))
                continue;

            colors[cp * 3 + 0] = static_cast<unsigned char> ((1.0f + normal.normal_x) * 127);
            colors[cp * 3 + 1] = static_cast<unsigned char> ((1.0f + normal.normal_y) * 127);
            colors[cp * 3 + 2] = static_cast<unsigned char> ((1.0f + normal.normal_z) * 127);
        }
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
        return (true);
    }

  protected:
    /** \brief Class getName method. */
    virtual std::string
    getName () const { return ("PointCloudNormalHandler"); }

  private:
    // Members derived from the base class
    using PointCloudColorHandler<PointT>::cloud_;
    using PointCloudColorHandler<PointT>::capable_;
    pcl::PointCloud<pcl::Normal>::ConstPtr normals_;
};

}
}
#endif // POINTCLOUDNORMALHANDLER_H

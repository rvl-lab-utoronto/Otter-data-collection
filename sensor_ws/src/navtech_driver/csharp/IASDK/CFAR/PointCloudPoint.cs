namespace Navtech.IASDK.CFAR
{

    /// <summary>
    /// A CFAR-produced point in an azimuth
    /// </summary>
    public struct PointCloudPoint
    {
        /// <summary>
        /// The range (in metres) of the point
        /// </summary>
        public double Range;

        /// <summary>
        /// The power (in dB) of the point
        /// </summary>
        public double Power;
    }

}
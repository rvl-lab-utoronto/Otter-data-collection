using System;
using System.Collections.Generic;
using System.Linq;

namespace Navtech.IASDK.CFAR
{
    /// <summary>
    /// Cell Averaging CFAR implementation
    /// </summary>
    /// <typeparam name="T">The type used in the data</typeparam>
    public class CellAverage<T>
    {
        private readonly ushort _startBin;
        private readonly ushort _windowSize;
        private readonly ushort _guardCells;
        private readonly T _threshold;

        private readonly Func<int, double> _toMetre;

        /// <summary>
        /// A cell averaging CFAR (CA/CFAR) class using the default conversion function
        /// </summary>
        /// <param name="startBin">The minimum bin from which cell averaging will be performed</param>
        /// <param name="windowSize">The size of the CFAR window</param>
        /// <param name="guardCells">Number of cells guarding training cells from signal</param>
        /// <param name="threshold">Value above the window's mean to identify a point</param>
        public CellAverage(
            ushort startBin,
            ushort windowSize,
            ushort guardCells,
            T threshold
        )
        {
            _startBin = startBin;
            _windowSize = windowSize;
            _guardCells = guardCells;
            _threshold = threshold;
            _toMetre = Convert.ToDouble;
        }



        /// <summary>
        /// A cell averaging CFAR (CA/CFAR) class
        /// </summary>
        /// <param name="startBin">The minimum bin from which cell averaging will be performed</param>
        /// <param name="windowSize">The size of the CFAR window</param>
        /// <param name="guardCells">Number of cells guarding training cells from signal</param>
        /// <param name="threshold">Value above the window's mean to identify a point</param>
        /// <param name="toMetre">Conversion function from type T to metres. </param>
        public CellAverage(
            ushort startBin,
            ushort windowSize,
            ushort guardCells,
            T threshold,
            Func<int, double> toMetre
            )
        {
            _startBin = startBin;
            _windowSize = windowSize;
            _guardCells = guardCells;
            _threshold = threshold;
            _toMetre = toMetre;
        }


        /// <summary>
        /// Find and return all results along the data as Points
        /// </summary>
        /// <param name="data">Input data</param>
        /// <returns>All Point cloud points in the data, as a List</returns>
        public List<PointCloudPoint> Points(List<T> data)
        {
            return FirstNPoints(data, data.Count);
        }


        /// <summary>
        /// Find and return the first N non-zero results as Points
        /// </summary>
        /// <param name="data">Input data</param>
        /// <param name="maxPoints">The maximum number of points to return</param>
        /// <returns>Up to N point cloud points of data, as a List</returns>
        public List<PointCloudPoint> FirstNPoints(List<T> data, int maxPoints)
        {
            var output = new List<PointCloudPoint>();

            var points = 0;
            var index = _startBin;
            var processedEnumerable = ProcessNextCell(data);
            while (processedEnumerable.MoveNext() && points < maxPoints)
            {
                if (processedEnumerable.Current > 0.0)
                {
                    output.Add(
                        new PointCloudPoint
                        {
                            Range = _toMetre(index),
                            Power = processedEnumerable.Current
                        });

                    points++;
                }

                index++;
            }

            return output;
        }


        /// <summary>
        /// Process all data using CA/CFAR
        /// </summary>
        /// <param name="data">Input data</param>
        /// <returns>CFAR result for each element in the data</returns>
        public List<double> Process(List<T> data)
        {
            return Process(data, data.Count);
        }


        /// <summary>
        /// Process data up until a certain size
        /// </summary>
        /// <param name="data">The input data</param>
        /// <param name="size">The maximum size to process</param>
        /// <returns>CFAR result for each element of the input up to the size</returns>
        public List<double> Process(List<T> data, int size)
        {
            var output = Enumerable.Range(0, size).Select(_ => 0.0).ToList();
            var enumerated = ProcessNextCell(data);
            var outputIndex = _startBin;
            while (enumerated.MoveNext() && outputIndex < size)
            {
                output[outputIndex] = enumerated.Current;
                outputIndex++;
            }
            return output;
        }


        /// <summary>
        /// Process the next CFAR cell of the given data
        /// </summary>
        /// <param name="data">The input data</param>
        /// <returns>Data above threshold, or 0.0</returns>
        private IEnumerator<double> ProcessNextCell(List<T> data)
        {
            for (var i = _startBin; i < data.Count; i++)
            {
                var lowerBegin = i - _windowSize / 2;
                var lowerEnd = i - _guardCells;

                var upperBegin = i + _guardCells + 1;
                var upperEnd = i + _windowSize / 2 + 1;

                if (lowerBegin < _startBin)
                {
                    lowerBegin = _startBin;
                    upperEnd = lowerBegin + _windowSize;
                }

                if (upperEnd > data.Count)
                {
                    upperEnd = data.Count;
                    lowerBegin = upperEnd - _windowSize;
                }

                if (lowerEnd < _startBin) lowerEnd = _startBin;
                if (upperBegin > data.Count) upperBegin = data.Count;

                var lowerSum = data.Slice(lowerBegin, lowerEnd - lowerBegin).Sum(v => Convert.ToDouble(v));

                var upperSum = data.Slice(upperBegin, upperEnd - upperBegin).Sum(v => Convert.ToDouble(v));

                var elements = lowerEnd - lowerBegin + upperEnd - upperBegin;
                var average = (upperSum + lowerSum) / elements;

                var next = Convert.ToDouble(data[i]);
                if (next > average + Convert.ToDouble(_threshold))
                {
                    yield return next;
                }
                else
                {
                    yield return 0.0;
                }

            }
        }
    }
}

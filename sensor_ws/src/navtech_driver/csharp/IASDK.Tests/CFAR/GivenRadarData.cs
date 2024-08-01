using Navtech.IASDK.CFAR;

namespace Navtech.IASDK.Tests.Cfar;

[Parallelizable]
[TestFixture]
public class GivenRadarData
{
    [Test]
    public void WhenConstructedShouldNotThrow()
    {
        Assert.DoesNotThrow(
            () =>
                // ReSharper disable once ObjectCreationAsStatement
                new CellAverage<double>(
                    0,
                    10,
                    2,
                    5.0
                )
            );

        Assert.DoesNotThrow(
            () =>
            
                // ReSharper disable once ObjectCreationAsStatement
                new CellAverage<double>(
                    0,
                    10,
                    2,
                    5.0,
                    b => 0.1752 * b
                )
    
            );
    }


    [Test]
    public void WhenGivenFftDataShouldIdentifyNPoints()
    {
        const double targetVal = 75.0;
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }

        testData[30] = targetVal;
        testData[60] = targetVal;
        testData[90] = targetVal;
        var cellAverage = new CellAverage<double>(0, 10, 2, 10.0);

        var nPoints = cellAverage.FirstNPoints(testData, 3);
        Assert.That(nPoints.Count == 3);
        Assert.That(Math.Abs(nPoints[0].Range - 30.0) < double.Epsilon);
        Assert.That(Math.Abs(nPoints[0].Power - targetVal) < double.Epsilon);
        Assert.That(Math.Abs(nPoints[1].Range - 60.0) < double.Epsilon);
        Assert.That(Math.Abs(nPoints[1].Power - targetVal) < double.Epsilon);
        Assert.That(Math.Abs(nPoints[2].Range - 90.0) < double.Epsilon);
        Assert.That(Math.Abs(nPoints[2].Power - targetVal) < double.Epsilon);
    }


    [Test]
    public void WhenGivenFftDataWithDataAboveThresholdShouldReturnPoints()
    {
        const double targetVal = 75.0;
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }

        testData[30] = targetVal;
        testData[60] = targetVal;
        testData[90] = targetVal;
        var cellAverage = new CellAverage<double>(0, 10, 2, 10.0);

        var processed = cellAverage.Points(testData);
        Assert.That(processed.Count == 3);
    }


    [Test]
    public void WhenProcessingAllPointsShouldReturnListWithNNonZero()
    {
        const double targetVal = 75.0;
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }
        testData[15] = targetVal;
        testData[30] = targetVal;
        testData[45] = targetVal;
        testData[60] = targetVal;
        testData[75] = targetVal;

        var cellAverage = new CellAverage<double>(0, 10, 2, 10.0);

        var allPoints = cellAverage.Process(testData);
        Assert.That(allPoints.Count == 100);
        Assert.That(allPoints.Select(x => x).Count(x => x > 0.0) == 5);
    }


    [Test]
    public void WhenProcessingAllPointsResultShouldBeSameLengthAsInput()
    {
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }

        var cellAverage = new CellAverage<double>(0, 10, 2, 10.0);
        var allPoints = cellAverage.Process(testData);
        Assert.That(allPoints.Count == testData.Count);
    }


    [Test]
    public void WhenProcessingNValuesShouldReturnPoints()
    {
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }

        var cellAverage = new CellAverage<double>(0, 10, 2, 10.0);
        var allPoints = cellAverage.Process(testData, 50);
        Assert.That(allPoints.Count == 50);
    }


    [Test]
    public void WhenUsingConversionFuncShouldCorrectlyConvert()
    {
        const double conversionFactor = 0.1752;
        const int position = 50;
        const double expected = position * conversionFactor;
        const double targetVal = 75.0;
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }

        testData[position] = targetVal;

        var cellAverage = new CellAverage<double>(
            0,
            10,
            2,
            10.0,
            b => conversionFactor * b
        );

        var points = cellAverage.FirstNPoints(testData, 1);
        Assert.That(points.Count == 1);
        Assert.That(Math.Abs(points[0].Range - expected) < double.Epsilon);
    }


    [Test]
    public void WhenGivenNonZeroMinBinShouldNotFindPointsBelowMinimum()
    {
        const double targetVal = 75.0;
        var testData = new List<double>();
        var rand = new Random();
        for (var i = 0; i < 100; i++)
        {
            testData.Add(55.0 + rand.NextDouble() * 5); // Values between 55.0 and 60.0, average should be 57.5 ish
        }

        testData[25] = targetVal;
        testData[50] = targetVal;
        testData[75] = targetVal;

        var cellAverage = new CellAverage<double>(
            30, // startBin
            10,
            2,
            10.0
        );

        var points = cellAverage.Points(testData);
        Assert.That(points.Count == 2);
        Assert.That(Math.Abs(points[0].Range - 50.0) < double.Epsilon);
        Assert.That(Math.Abs(points[1].Range - 75.0) < double.Epsilon);
    }
}
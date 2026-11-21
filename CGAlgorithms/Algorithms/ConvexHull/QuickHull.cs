using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class QuickHull : Algorithm
    {
        private void FindHull(Point a, Point b, List<Point> region, List<Point> outPoints)
        {
            if (region.Count == 0) return;

            Point ab = a.Vector(b);
            Point far = region[0];
            double maxDist = -1;
            double minProj = double.MaxValue;

            foreach (Point p in region)
            {
                Point ap = a.Vector(p);
                double dist = Math.Abs(HelperMethods.CrossProduct(ab, ap));
                double proj = ab.X * ap.X + ab.Y * ap.Y;

                if (dist > maxDist || (dist == maxDist && proj < minProj))
                {
                    maxDist = dist;
                    minProj = proj;
                    far = p;
                }
            }

            var right1 = new List<Point>();
            var right2 = new List<Point>();
            Line l1 = new Line(a, far);
            Line l2 = new Line(far, b);

            foreach (Point p in region)
            {
                if (p.Equals(far)) continue;
                if (HelperMethods.CheckTurn(l1, p) == Enums.TurnType.Right)
                    right1.Add(p);
                else if (HelperMethods.CheckTurn(l2, p) == Enums.TurnType.Right)
                    right2.Add(p);
            }

            FindHull(a, far, right1, outPoints);
            outPoints.Add(far);
            FindHull(far, b, right2, outPoints);
        }

        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            var uniquePointsTmp = new HashSet<Tuple<double, double>>();
            var pts = new List<Point>();
            foreach (Point p in points)
            {
                var cur = new Tuple<double, double>(p.X, p.Y);
                if (uniquePointsTmp.Contains(cur)) continue;
                uniquePointsTmp.Add(cur);
                pts.Add(p);
            }

            if (pts.Count <= 2)
            {
                outPoints = new List<Point>(pts);
                return;
            }

            // Find the extreme points in the horizontal and vertical directions
            Point bottomMinX = pts[0], bottomMaxX = pts[0];
            Point rightMinY = pts[0], rightMaxY = pts[0];
            Point topMaxX = pts[0], topMinX = pts[0];
            Point leftMaxY = pts[0], leftMinY = pts[0];

            foreach (Point p in pts)
            {
                if (p.Y < bottomMinX.Y || (p.Y == bottomMinX.Y && p.X < bottomMinX.X)) bottomMinX = p;
                if (p.Y < bottomMaxX.Y || (p.Y == bottomMaxX.Y && p.X > bottomMaxX.X)) bottomMaxX = p;

                if (p.X > rightMinY.X || (p.X == rightMinY.X && p.Y < rightMinY.Y)) rightMinY = p;
                if (p.X > rightMaxY.X || (p.X == rightMaxY.X && p.Y > rightMaxY.Y)) rightMaxY = p;

                if (p.Y > topMaxX.Y || (p.Y == topMaxX.Y && p.X > topMaxX.X)) topMaxX = p;
                if (p.Y > topMinX.Y || (p.Y == topMinX.Y && p.X < topMinX.X)) topMinX = p;

                if (p.X < leftMaxY.X || (p.X == leftMaxY.X && p.Y > leftMaxY.Y)) leftMaxY = p;
                if (p.X < leftMinY.X || (p.X == leftMinY.X && p.Y < leftMinY.Y)) leftMinY = p;
            }

            // Compute the initial CCW convex hull of these extreme points.
            var candidates = new[]
            {
                bottomMinX, bottomMaxX,
                rightMinY, rightMaxY,
                topMaxX, topMinX,
                leftMaxY, leftMinY
            };

            var initialHull = new List<Point>(8);
            foreach (Point c in candidates)
            {
                if (initialHull.Count == 0 || !initialHull[initialHull.Count - 1].Equals(c))
                    initialHull.Add(c);
            }
            if (initialHull.Count > 1 && initialHull[initialHull.Count - 1].Equals(initialHull[0]))
                initialHull.RemoveAt(initialHull.Count - 1);

            // Classify remaining points into exterior regions and prune interior points.
            int m = initialHull.Count;
            var regions = new List<Point>[m];
            var edges = new Line[m];
            var isDiagonal = new bool[m];

            for (int i = 0; i < m; i++)
            {
                regions[i] = new List<Point>();
                edges[i] = new Line(initialHull[i], initialHull[(i + 1) % m]);
                isDiagonal[i] = edges[i].Start.X != edges[i].End.X && edges[i].Start.Y != edges[i].End.Y;
            }

            foreach (Point p in pts)
            {
                for (int i = 0; i < m; i++)
                {
                    if (isDiagonal[i] && HelperMethods.CheckTurn(edges[i], p) == Enums.TurnType.Right)
                    {
                        regions[i].Add(p);
                        break;
                    }
                }
            }

            outPoints = new List<Point>();
            for (int i = 0; i < m; i++)
            {
                outPoints.Add(initialHull[i]);
                FindHull(initialHull[i], initialHull[(i + 1) % m], regions[i], outPoints);
            }
        }

        public override string ToString()
        {
            return "Convex Hull - Quick Hull";
        }
    }
}

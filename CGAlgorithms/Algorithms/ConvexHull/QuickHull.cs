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
        private void FindHull(Point a, Point b, List<Point> pts, List<Point> outPoints)
        {
            if (pts.Count == 0) return;

            Point ab = a.Vector(b);
            Point far = pts[0];
            double maxDist = -1;
            double minProj = double.MaxValue;

            foreach (Point p in pts)
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

            outPoints.Add(far);

            var left1 = new List<Point>();
            var left2 = new List<Point>();
            Line l1 = new Line(a, far);
            Line l2 = new Line(far, b);

            foreach (Point p in pts)
            {
                if (p.Equals(far)) continue;
                if (HelperMethods.CheckTurn(l1, p) == Enums.TurnType.Left)
                    left1.Add(p);
                else if (HelperMethods.CheckTurn(l2, p) == Enums.TurnType.Left)
                    left2.Add(p);
            }

            FindHull(a, far, left1, outPoints);
            FindHull(far, b, left2, outPoints);
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

            Point minP = pts[0], maxP = pts[0];
            foreach (Point p in pts)
            {
                if (p.X < minP.X || (p.X == minP.X && p.Y < minP.Y)) minP = p;
                if (p.X > maxP.X || (p.X == maxP.X && p.Y > maxP.Y)) maxP = p;
            }

            outPoints.Add(minP);
            outPoints.Add(maxP);

            var left = new List<Point>();
            var right = new List<Point>();
            Line mainLine = new Line(minP, maxP);

            foreach (Point p in pts)
            {
                if (p.Equals(minP) || p.Equals(maxP)) continue;
                var dir = HelperMethods.CheckTurn(mainLine, p);
                if (dir == Enums.TurnType.Left) left.Add(p);
                else if (dir == Enums.TurnType.Right) right.Add(p);
            }

            FindHull(minP, maxP, left, outPoints);
            FindHull(maxP, minP, right, outPoints);
        }

        public override string ToString()
        {
            return "Convex Hull - Quick Hull";
        }
    }
}

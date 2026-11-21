using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class QuickHull : Algorithm
    {
        private bool ContainsPoint(List<Point> list, Point target)
        {
            for (int i = 0; i < list.Count; i++)
            {
                if (list[i].Equals(target))
                    return true;
            }
            return false;
        }

        private void BuildHull(Point a, Point b, List<Point> candidates, List<Point> hull)
        {
            if (candidates.Count == 0)
                return;

            Point ab = a.Vector(b);
            Point farthest = null;
            double bestDist = Constants.Epsilon;

            foreach (Point p in candidates)
            {
                if (p.Equals(a) || p.Equals(b))
                    continue;

                // Cross product magnitude is proportional to perpendicular distance from segment AB
                double dist = Math.Abs(HelperMethods.CrossProduct(ab, a.Vector(p)));
                if (dist > bestDist + Constants.Epsilon)
                {
                    bestDist = dist;
                    farthest = p;
                }
            }

            if (farthest == null)
                return;

            Line firstEdge = new Line(a, farthest);
            Line secondEdge = new Line(farthest, b);

            var leftOfFirst = new List<Point>();
            var leftOfSecond = new List<Point>();

            foreach (Point p in candidates)
            {
                if (p.Equals(farthest) || p.Equals(a) || p.Equals(b))
                    continue;

                if (HelperMethods.CheckTurn(firstEdge, p) == Enums.TurnType.Right)
                    leftOfFirst.Add(p);
                else if (HelperMethods.CheckTurn(secondEdge, p) == Enums.TurnType.Right)
                    leftOfSecond.Add(p);
            }

            BuildHull(a, farthest, leftOfFirst, hull);
            if (!ContainsPoint(hull, farthest))
                hull.Add(farthest);
            BuildHull(farthest, b, leftOfSecond, hull);
        }

        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            outPoints = new List<Point>();

            var unique = new List<Point>();
            foreach (Point p in points)
            {
                if (!ContainsPoint(unique, p))
                    unique.Add(p);
            }

            if (unique.Count == 0)
                return;

            if (unique.Count <= 2)
            {
                outPoints.AddRange(unique);
                return;
            }

            // Find the extreme points in the horizontal direction
            Point minX = unique[0];
            Point maxX = unique[0];

            foreach (Point p in unique)
            {
                if (p.X < minX.X - Constants.Epsilon || (Math.Abs(p.X - minX.X) <= Constants.Epsilon && p.Y < minX.Y))
                    minX = p;

                if (p.X > maxX.X + Constants.Epsilon || (Math.Abs(p.X - maxX.X) <= Constants.Epsilon && p.Y > maxX.Y))
                    maxX = p;
            }

            if (minX.Equals(maxX))
            {
                outPoints.Add(minX);
                return;
            }

            // Classify remaining points into exterior regions and prune interior points.
            Line topLine = new Line(minX, maxX);
            Line bottomLine = new Line(maxX, minX);

            var upperSet = new List<Point>();
            var lowerSet = new List<Point>();

            foreach (Point p in unique)
            {
                if (p.Equals(minX) || p.Equals(maxX))
                    continue;

                if (HelperMethods.CheckTurn(topLine, p) == Enums.TurnType.Right)
                    upperSet.Add(p);
                else if (HelperMethods.CheckTurn(bottomLine, p) == Enums.TurnType.Right)
                    lowerSet.Add(p);
            }

            outPoints.Add(minX);
            BuildHull(minX, maxX, upperSet, outPoints);

            if (!ContainsPoint(outPoints, maxX))
                outPoints.Add(maxX);

            BuildHull(maxX, minX, lowerSet, outPoints);
        }

        public override string ToString()
        {
            return "Convex Hull - Quick Hull";
        }
    }
}
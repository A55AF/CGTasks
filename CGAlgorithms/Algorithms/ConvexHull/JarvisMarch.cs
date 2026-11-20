using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class JarvisMarch : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            // 1. Edge cases
            if (points == null || points.Count == 0)
            {
                return;
            }

            if (outPoints == null)
            {
                outPoints = new List<Point>();
            }
            else
            {
                outPoints.Clear();
            }

            // Handle small sets of points (<= 3 points)
            if (points.Count <= 3)
            {
                // Eliminate duplicates (if any) or add points directly to outPoints
                for (int i = 0; i < points.Count; i++)
                {
                    bool isDuplicate = false;
                    for (int j = 0; j < outPoints.Count; j++)
                    {
                        if (points[i].Equals(outPoints[j]))
                        {
                            isDuplicate = true;
                            break;
                        }
                    }

                    if (!isDuplicate)
                    {
                        outPoints.Add(points[i]);
                    }
                }
                return;
            }

            // 2. Step 1: Lexicographically smallest point
            // Find point with minimum X coordinate (tie-break with minimum Y coordinate)
            Point p0 = points[0];
            for (int i = 1; i < points.Count; i++)
            {
                if (points[i].X < p0.X)
                {
                    p0 = points[i];
                }
                else if (points[i].X == p0.X && points[i].Y < p0.Y)
                {
                    p0 = points[i];
                }
            }

            outPoints.Add(p0);

            // 3. Step 2: Wrapping loop (detecting angularly rightmost point)
            Point v = p0;

            while (true)
            {
                // Pick an initial candidate point pj from points that is not equal to v
                Point pj = null;
                for (int i = 0; i < points.Count; i++)
                {
                    if (!points[i].Equals(v))
                    {
                        pj = points[i];
                        break;
                    }
                }

                // If all points are identical to v, no other candidate exists
                if (pj == null)
                {
                    break;
                }

                // Iterate over all points pk in points to find the most clockwise / rightmost turn
                for (int i = 0; i < points.Count; i++)
                {
                    Point pk = points[i];

                    // Skip evaluating the current hull vertex itself
                    if (pk.Equals(v))
                    {
                        continue;
                    }

                    // Evaluate orientation of triplet (v, pj, pk) using HelperMethods.CheckTurn
                    Enums.TurnType turn = HelperMethods.CheckTurn(new Line(v, pj), pk);

                    if (turn == Enums.TurnType.Right)
                    {
                        // pk is angularly more to the right of ray (v -> pj)
                        pj = pk;
                    }
                    else if (turn == Enums.TurnType.Colinear)
                    {
                        // 4. Collinear tie-breaking:
                        // If v, pj, and pk are collinear, choose the point farthest from v
                        if (v.Vector(pk).Magnitude() > v.Vector(pj).Magnitude())
                        {
                            pj = pk;
                        }
                    }
                }

                // 5. Termination check:
                // If the detected rightmost point equals the starting point, the hull has wrapped around
                if (pj.Equals(outPoints[0]))
                {
                    break;
                }

                // Add the next hull vertex and advance
                outPoints.Add(pj);
                v = pj;
            }
        }

        public override string ToString()
        {
            return "Convex Hull - Jarvis March";
        }
    }
}
using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class JarvisMarch : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            // base cases
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

            // if 3 or fewer points, just remove duplicates and return
            if (points.Count <= 3)
            {
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

            // find the leftmost point (lowest y to break ties)
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

            // gift wrapping loop
            Point v = p0;

            while (true)
            {
                // find an initial point different from current vertex v
                Point pj = null;
                for (int i = 0; i < points.Count; i++)
                {
                    if (!points[i].Equals(v))
                    {
                        pj = points[i];
                        break;
                    }
                }

                if (pj == null)
                {
                    break;
                }

                // find the most rightward turn (clockwise)
                for (int i = 0; i < points.Count; i++)
                {
                    Point pk = points[i];

                    if (pk.Equals(v))
                    {
                        continue;
                    }

                    Enums.TurnType turn = HelperMethods.CheckTurn(new Line(v, pj), pk);

                    if (turn == Enums.TurnType.Right)
                    {
                        // pk is more to the right, update candidate
                        pj = pk;
                    }
                    else if (turn == Enums.TurnType.Colinear)
                    {
                        // collinear points: pick the farthest one
                        if (v.Vector(pk).Magnitude() > v.Vector(pj).Magnitude())
                        {
                            pj = pk;
                        }
                    }
                }

                // stop once we loop back to the start point
                if (pj.Equals(outPoints[0]))
                {
                    break;
                }

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
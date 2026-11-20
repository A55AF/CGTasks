using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class ExtremePoints : Algorithm
    {
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
            if (pts.Count == 3)
            {
                pts.Sort((a, b) =>
                {
                    int cmpX = a.X.CompareTo(b.X);
                    return cmpX != 0 ? cmpX : a.Y.CompareTo(b.Y);
                });

                Point p1 = pts[0];
                Point p2 = pts[1];
                Point p3 = pts[2];
                if (HelperMethods.CheckTurn(new Line(p1, p2), p3) == Enums.TurnType.Colinear)
                {
                    outPoints = new List<Point> { pts.First(), pts.Last() };
                    return;
                }

                outPoints = new List<Point> { p1, p2, p3 };
                return;
            }

            for (int i = 0; i < pts.Count(); i++)
            {
                Point cand = pts[i];
                bool interior = false;
                for (int j = 0; j < pts.Count(); j++)
                {
                    if (j == i) continue;

                    Point p1 = pts[j];
                    for (int k = j + 1; k < pts.Count(); k++)
                    {
                        if (k == j || k == i) continue;

                        for (int l = k + 1; l < pts.Count(); l++)
                        {
                            if (l == k || l == j || l == i) continue;
                            Point p2 = pts[k];
                            Point p3 = pts[l];
                            if (HelperMethods.CheckTurn(new Line(p1, p2), p3) == Enums.TurnType.Colinear)
                            {
                                if (HelperMethods.CheckTurn(new Line(p1, p2), cand) == Enums.TurnType.Colinear
                                    && (HelperMethods.PointOnSegment(cand, p1, p2)
                                    || HelperMethods.PointOnSegment(cand, p1, p3)
                                    || HelperMethods.PointOnSegment(cand, p2, p3)))
                                {
                                    interior = true;
                                    break;
                                }
                                else
                                {
                                    continue;
                                }
                            }

                            if (HelperMethods.CheckTurn(new Line(p1, p2), p3) == Enums.TurnType.Right)
                            {
                                var tmp = p2;
                                p2 = p3;
                                p3 = tmp;
                            }

                            if (HelperMethods.CheckTurn(new Line(p1, p2), cand) != Enums.TurnType.Right
                                && HelperMethods.CheckTurn(new Line(p2, p3), cand) != Enums.TurnType.Right
                                && HelperMethods.CheckTurn(new Line(p3, p1), cand) != Enums.TurnType.Right)
                            {
                                interior = true;
                                break;
                            }
                        }
                        if (interior) break;
                    }
                    if (interior) break;
                }
                if (interior) continue;
                outPoints.Add(cand);
            }
        }

        public override string ToString()
        {
            return "Convex Hull - Extreme Points";
        }
    }
}

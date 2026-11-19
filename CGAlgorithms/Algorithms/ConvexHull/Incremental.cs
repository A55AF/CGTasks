using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class Incremental : Algorithm
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

            pts.Sort((a, b) =>
            {
                int cmpX = a.X.CompareTo(b.X);
                return cmpX != 0 ? cmpX : a.Y.CompareTo(b.Y);
            });

            Point p1 = pts[0];
            Point p2 = pts[1];
            Point p3 = null;
            for (int i = 2; i < pts.Count(); i++)
            {
                if (HelperMethods.CheckTurn(new Line(p1, p2), pts[i]) != Enums.TurnType.Colinear)
                {
                    p3 = pts[i];
                    break;
                }
            }

            if (p3 == null)
            {
                outPoints = new List<Point> { pts.First(), pts.Last() };
                return;
            }

            if (HelperMethods.CheckTurn(new Line(p1, p2), p3) == Enums.TurnType.Right)
            {
                Point tmp = p2;
                p2 = p3;
                p3 = tmp;
            }

            var hull = new List<Point> { p1, p2, p3 };
            for (int i = 2; i < pts.Count(); i++)
            {
                Point p = pts[i];
                int h = hull.Count();
                if (p == p3) continue;

                bool[] visible = new bool[h];
                bool is_exterior = false;
                for (int j = 0; j < h; j++)
                {
                    var turn = HelperMethods.CheckTurn(new Line(hull[j], hull[(j + 1) % h]), p);
                    if (turn != Enums.TurnType.Left)
                    {
                        visible[j] = true;
                    }
                    is_exterior |= visible[j];
                }

                if (!is_exterior) continue;

                int l = 0;
                while (!(visible[l] && !visible[(l - 1 + h) % h]))
                {
                    l = (l + 1) % h;
                }
                int r = l;
                while (visible[r])
                {
                    r = (r + 1) % h;
                }

                var newHull = new List<Point>();
                int cur = r;
                while (true)
                {
                    newHull.Add(hull[cur]);
                    if (cur == l) break;
                    cur = (cur + 1) % h;
                }
                newHull.Add(p);
                hull = newHull;
            }

            outPoints = hull;
        }

        public override string ToString()
        {
            return "Convex Hull - Incremental";
        }
    }
}

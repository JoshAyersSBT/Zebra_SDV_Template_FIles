# SDV Color Map

Reference from `SDV_GM_v3.pdf`, Fig. 5 "Grand loop Ride".

Rows are `A` to `E` from top to bottom. Columns are `1` to `5` from left to right.

## Column Crossings

When moving left or right between columns:

| Crossing | Line color |
| --- | --- |
| `1 <-> 2` | green |
| `2 <-> 3` | cyan |
| `3 <-> 4` | brown |
| `4 <-> 5` | magenta |

## Row Crossings

When moving up or down between rows:

| Crossing | Line color |
| --- | --- |
| `A <-> B` | yellow |
| `B <-> C` | blue |
| `C <-> D` | red |
| `D <-> E` | purple |

## Format A Clockwise Outer Loop

| Move | Expected line |
| --- | --- |
| `A1 -> A2` | green |
| `A2 -> A3` | cyan |
| `A3 -> A4` | brown |
| `A4 -> A5` | magenta |
| `A5 -> B5` | yellow |
| `B5 -> C5` | blue |
| `C5 -> D5` | red |
| `D5 -> E5` | purple |
| `E5 -> E4` | magenta |
| `E4 -> E3` | brown |
| `E3 -> E2` | cyan |
| `E2 -> E1` | green |
| `E1 -> D1` | purple |
| `D1 -> C1` | red |
| `C1 -> B1` | blue |
| `B1 -> A1` | yellow |

For counterclockwise driving, use the same line colors in reverse order.

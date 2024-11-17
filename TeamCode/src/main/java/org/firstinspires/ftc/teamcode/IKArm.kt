package org.firstinspires.ftc.teamcode

import kotlin.math.*

class IKArm {

    // Denavit-Hartenberg parameters
    private val a = doubleArrayOf(0.0, 0.2, 0.2, 0.0, 0.0, 0.0)
    private val alpha = doubleArrayOf(PI / 2, 0.0, 0.0, PI / 2, -PI / 2, 0.0)
    private val d = doubleArrayOf(0.1, 0.0, 0.0, 0.0, 0.0, 0.1)

    var jointAngles = DoubleArray(6) // Initial joint angles

    // Forward Kinematics: Compute end effector position given joint angles
    fun forwardKinematics(jointAngles: DoubleArray): DoubleArray {
        var T = arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 1.0)
        )

        for (i in 0 until 6) {
            val theta = jointAngles[i]
            val Ai = computeDHMatrix(a[i], alpha[i], d[i], theta)
            T = matrixMultiply(T, Ai)
        }

        return doubleArrayOf(T[0][3], T[1][3], T[2][3])
    }

    // Compute Jacobian matrix
    fun computeJacobian(jointAngles: DoubleArray): Array<DoubleArray> {
        val jacobian = Array(3) { DoubleArray(6) }
        val positions = Array(7) { DoubleArray(3) }
        val zAxes = Array(7) { DoubleArray(3) }

        var T = arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 1.0)
        )

        positions[0] = doubleArrayOf(0.0, 0.0, 0.0)
        zAxes[0] = doubleArrayOf(0.0, 0.0, 1.0)

        for (i in 0 until 6) {
            val theta = jointAngles[i]
            val Ai = computeDHMatrix(a[i], alpha[i], d[i], theta)
            T = matrixMultiply(T, Ai)

            positions[i + 1] = doubleArrayOf(T[0][3], T[1][3], T[2][3])
            zAxes[i + 1] = doubleArrayOf(T[0][2], T[1][2], T[2][2])
        }

        val pe = positions[6]

        for (i in 0 until 6) {
            val zi = zAxes[i]
            val pi = positions[i]
            val r = subtractVectors(pe, pi)
            val cross = crossProduct(zi, r)

            jacobian[0][i] = cross[0]
            jacobian[1][i] = cross[1]
            jacobian[2][i] = cross[2]
        }

        return jacobian
    }

    // Inverse Kinematics using Damped Least Squares
    fun inverseKinematics(targetX: Double, targetY: Double, targetZ: Double): DoubleArray {
        val maxIterations = 1000
        val tolerance = 1e-6
        val alpha = 0.1
        val lambda = 0.01

        var jointAngles = this.jointAngles.copyOf()

        for (iter in 0 until maxIterations) {
            val currentPos = forwardKinematics(jointAngles)
            val error = doubleArrayOf(
                    targetX - currentPos[0],
                    targetY - currentPos[1],
                    targetZ - currentPos[2]
            )

            if (norm(error) < tolerance) break

            val jacobian = computeJacobian(jointAngles)
            val jacobianPseudoInverse = dampedLeastSquaresInverse(jacobian, lambda)
            val deltaTheta = multiplyMatrixVector(jacobianPseudoInverse, error)

            for (i in 0 until 6) {
                jointAngles[i] += alpha * deltaTheta[i]
            }
        }

        return jointAngles
    }

    // Compute DH Transformation Matrix
    private fun computeDHMatrix(a: Double, alpha: Double, d: Double, theta: Double): Array<DoubleArray> {
        val sa = sin(alpha)
        val ca = cos(alpha)
        val st = sin(theta)
        val ct = cos(theta)

        return arrayOf(
                doubleArrayOf(ct, -st * ca, st * sa, a * ct),
                doubleArrayOf(st, ct * ca, -ct * sa, a * st),
                doubleArrayOf(0.0, sa, ca, d),
                doubleArrayOf(0.0, 0.0, 0.0, 1.0)
        )
    }

    // Matrix Multiplication
    private fun matrixMultiply(A: Array<DoubleArray>, B: Array<DoubleArray>): Array<DoubleArray> {
        val result = Array(4) { DoubleArray(4) }
        for (i in 0 until 4) {
            for (j in 0 until 4) {
                var sum = 0.0
                for (k in 0 until 4) {
                    sum += A[i][k] * B[k][j]
                }
                result[i][j] = sum
            }
        }
        return result
    }

    // Vector Subtraction
    private fun subtractVectors(a: DoubleArray, b: DoubleArray): DoubleArray {
        return doubleArrayOf(a[0] - b[0], a[1] - b[1], a[2] - b[2])
    }

    // Cross Product
    private fun crossProduct(a: DoubleArray, b: DoubleArray): DoubleArray {
        return doubleArrayOf(
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        )
    }

    // Norm of a Vector
    private fun norm(vector: DoubleArray): Double {
        return sqrt(vector.map { it * it }.sum())
    }

    // Damped Least Squares Inverse
    private fun dampedLeastSquaresInverse(jacobian: Array<DoubleArray>, lambda: Double): Array<DoubleArray> {
        val jt = transposeMatrix(jacobian)
        val jjt = matrixMultiply(jacobian, jt)
        val identity = Array(jjt.size) { DoubleArray(jjt.size) { 0.0 } }
        for (i in identity.indices) {
            identity[i][i] = 1.0
        }

        val dampingMatrix = Array(jjt.size) { DoubleArray(jjt.size) }
        for (i in dampingMatrix.indices) {
            for (j in dampingMatrix[i].indices) {
                dampingMatrix[i][j] = jjt[i][j] + lambda * lambda * identity[i][j]
            }
        }

        val inverseDampingMatrix = invertMatrix(dampingMatrix)
        return matrixMultiply(jt, inverseDampingMatrix)
    }

    // Transpose of a Matrix
    private fun transposeMatrix(matrix: Array<DoubleArray>): Array<DoubleArray> {
        val rows = matrix.size
        val cols = matrix[0].size
        val transposed = Array(cols) { DoubleArray(rows) }
        for (i in 0 until rows) {
            for (j in 0 until cols) {
                transposed[j][i] = matrix[i][j]
            }
        }
        return transposed
    }

    // Multiply Matrix and Vector
    private fun multiplyMatrixVector(matrix: Array<DoubleArray>, vector: DoubleArray): DoubleArray {
        val result = DoubleArray(matrix.size)
        for (i in matrix.indices) {
            var sum = 0.0
            for (j in vector.indices) {
                sum += matrix[i][j] * vector[j]
            }
            result[i] = sum
        }
        return result
    }

    // Invert a Matrix using Gaussian Elimination
    private fun invertMatrix(matrix: Array<DoubleArray>): Array<DoubleArray> {
        val n = matrix.size
        val augmented = Array(n) { DoubleArray(2 * n) }
        for (i in 0 until n) {
            for (j in 0 until n) {
                augmented[i][j] = matrix[i][j]
            }
            augmented[i][n + i] = 1.0
        }

        for (i in 0 until n) {
            var maxRow = i
            for (k in i + 1 until n) {
                if (abs(augmented[k][i]) > abs(augmented[maxRow][i])) {
                    maxRow = k
                }
            }

            val temp = augmented[i]
            augmented[i] = augmented[maxRow]
            augmented[maxRow] = temp

            for (k in i + 1 until n) {
                val c = -augmented[k][i] / augmented[i][i]
                for (j in i until 2 * n) {
                    if (i == j) {
                        augmented[k][j] = 0.0
                    } else {
                        augmented[k][j] += c * augmented[i][j]
                    }
                }
            }
        }

        for (i in n - 1 downTo 0) {
            val c = augmented[i][i]
            for (j in 0 until 2 * n) {
                augmented[i][j] /= c
            }
            for (k in 0 until i) {
                val c2 = -augmented[k][i]
                for (j in 0 until 2 * n) {
                    augmented[k][j] += c2 * augmented[i][j]
                }
            }
        }

        val inverse = Array(n) { DoubleArray(n) }
        for (i in 0 until n) {
            for (j in 0 until n) {
                inverse[i][j] = augmented[i][n + j]
            }
        }
        return inverse
    }
}
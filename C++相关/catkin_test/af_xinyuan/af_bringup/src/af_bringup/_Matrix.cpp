#include "_Matrix.h"

//�����෽��
//��ʼ��
_Matrix::_Matrix(int mm,int nn)
{
	m = mm;
	n = nn;
}

//����m
void _Matrix::set_m(int mm)
{
	m = mm;
}

//����n
void _Matrix::set_n(int nn)
{
	n = nn;
}

//��ʼ��
void _Matrix::init_matrix()
{
	arr = new float[m * n];
}

//�ͷ�
void _Matrix::free_matrix()
{
	delete []arr;
}

//��ȡi,j���������
//ʧ�ܷ���-31415,�ɹ�����ֵ
float _Matrix::read(int i,int j)
{
	if (i >= m || j >= n)
	{
		return -31415;
	}
	
	return *(arr + i * n + j);
}

//д��i,j���������
//ʧ�ܷ���-1,�ɹ�����1
int _Matrix::write(int i,int j,float val)
{
	if (i >= m || j >= n)
	{
		return -1;
	}
	
	*(arr + i * n + j) = val;
	return 1;
}

//���������෽��
//��ʼ��
_Matrix_Calc::_Matrix_Calc()
{
}

//C = A + B
//�ɹ�����1,ʧ�ܷ���-1
int _Matrix_Calc::add(_Matrix *A,_Matrix *B,_Matrix *C)
{
	int i = 0;
	int j = 0;
	
	//�ж��Ƿ��������
	if (A->m != B->m || A->n != B->n || \
		A->m != C->m || A->n != C->n)
	{
		return -1;
	}
	//����
	for (i = 0;i < C->m;i++)
	{
		for (j = 0;j < C->n;j++)
		{
			C->write(i,j,A->read(i,j) + B->read(i,j));
		}
	}
	
	return 1;
}

//C = A - B
//�ɹ�����1,ʧ�ܷ���-1
int _Matrix_Calc::subtract(_Matrix *A,_Matrix *B,_Matrix *C)
{
	int i = 0;
	int j = 0;
	
	//�ж��Ƿ��������
	if (A->m != B->m || A->n != B->n || \
		A->m != C->m || A->n != C->n)
	{
		return -1;
	}
	//����
	for (i = 0;i < C->m;i++)
	{
		for (j = 0;j < C->n;j++)
		{
			C->write(i,j,A->read(i,j) - B->read(i,j));
		}
	}
	
	return 1;
}

//C = A * B
//�ɹ�����1,ʧ�ܷ���-1
int _Matrix_Calc::multiply(_Matrix *A,_Matrix *B,_Matrix *C)
{
	int i = 0;
	int j = 0;
	int k = 0;
	float temp = 0;
	
	//�ж��Ƿ��������
	if (A->m != C->m || B->n != C->n || \
		A->n != B->m)
	{
		return -1;
	}
	//����
	for (i = 0;i < C->m;i++)
	{
		for (j = 0;j < C->n;j++)
		{
			temp = 0;
			for (k = 0;k < A->n;k++)
			{
				temp += A->read(i,k) * B->read(k,j);
			}
			C->write(i,j,temp);
		}
	}
	
	return 1;
}

//����ʽ��ֵ,ֻ�ܼ���2 * 2,3 * 3
//ʧ�ܷ���-31415,�ɹ�����ֵ
float _Matrix_Calc::det(_Matrix *A)
{
	float value = 0;
	
	//�ж��Ƿ��������
	if (A->m != A->n || (A->m != 2 && A->m != 3))
	{
		return -31415;
	}
	//����
	if (A->m == 2)
	{
		value = A->read(0,0) * A->read(1,1) - A->read(0,1) * A->read(1,0);
	}
	else
	{
		value = A->read(0,0) * A->read(1,1) * A->read(2,2) + \
				A->read(0,1) * A->read(1,2) * A->read(2,0) + \
				A->read(0,2) * A->read(1,0) * A->read(2,1) - \
				A->read(0,0) * A->read(1,2) * A->read(2,1) - \
				A->read(0,1) * A->read(1,0) * A->read(2,2) - \
				A->read(0,2) * A->read(1,1) * A->read(2,0);
	}
	
	return value;
}

//��ת�þ���,B = AT
//�ɹ�����1,ʧ�ܷ���-1
int _Matrix_Calc::transpos(_Matrix *A,_Matrix *B)
{
	int i = 0;
	int j = 0;
	
	//�ж��Ƿ��������
	if (A->m != B->n || A->n != B->m)
	{
		return -1;
	}
	//����
	for (i = 0;i < B->m;i++)
	{
		for (j = 0;j < B->n;j++)
		{
			B->write(i,j,A->read(j,i));
		}
	}
	
	return 1;
}

//��ӡ2ά����
void _Matrix_Calc::printf_matrix(_Matrix *A)
{
	int i = 0;
	int j = 0;
	int m = 0;
	int n = 0;
	
	m = A->m;
	n = A->n;
	for (i = 0;i < m;i++)
	{
		for (j = 0;j < n;j++)
		{
			printf("%f ",A->read(i,j));
		}
		printf("\n");
	}
}

//�������,B = A^(-1)
//�ɹ�����1,ʧ�ܷ���-1
int _Matrix_Calc::inverse(_Matrix *A,_Matrix *B)
{
	int i = 0;
	int j = 0;
	int k = 0;
	_Matrix m(A->m,2 * A->m);
	float temp = 0;
	float b = 0;
	
	//�ж��Ƿ��������
	if (A->m != A->n || B->m != B->n || A->m != B->m)
	{
		return -1;
	}
	
	//�����2ά����3ά������ʽ�ж��Ƿ����
	if (A->m == 2 || A->m == 3)
	{
		if (det(A) == 0)
		{
			return -1;
		}
	}
	
	//�������m = A | B��ʼ��
	m.init_matrix();
	for (i = 0;i < m.m;i++)
	{
		for (j = 0;j < m.n;j++)
		{
			if (j <= A->n - 1)
			{
				m.write(i,j,A->read(i,j));
			}
			else
			{
				if (i == j - A->n)
				{
					m.write(i,j,1);
				}
				else
				{
					m.write(i,j,0);
				}
			}
		}
	}
	
	//��˹��Ԫ
	//�任������
	for (k = 0;k < m.m - 1;k++)
	{
		//�������Ϊk,k����Ϊ0,���б任
		if (m.read(k,k) == 0)
		{
			for (i = k + 1;i < m.m;i++)
			{
				if (m.read(i,k) != 0)
				{
					break;
				}
			}
			if (i >= m.m)
			{
				return -1;
			}
			else
			{
				//������
				for (j = 0;j < m.n;j++)
				{
					temp = m.read(k,j);
					m.write(k,j,m.read(k + 1,j));
					m.write(k + 1,j,temp);
				}
			}
		}
		
		//��Ԫ
		for (i = k + 1;i < m.m;i++)
		{
			//��ñ���
			b = m.read(i,k) / m.read(k,k);
			//�б任
			for (j = 0;j < m.n;j++)
			{
				temp = m.read(i,j) - b * m.read(k,j);
				m.write(i,j,temp);
			}
		}
	}
	//�任������
	for (k = m.m - 1;k > 0;k--)
	{
		//�������Ϊk,k����Ϊ0,���б任
		if (m.read(k,k) == 0)
		{
			for (i = k + 1;i < m.m;i++)
			{
				if (m.read(i,k) != 0)
				{
					break;
				}
			}
			if (i >= m.m)
			{
				return -1;
			}
			else
			{
				//������
				for (j = 0;j < m.n;j++)
				{
					temp = m.read(k,j);
					m.write(k,j,m.read(k + 1,j));
					m.write(k + 1,j,temp);
				}
			}
		}
		
		//��Ԫ
		for (i = k - 1;i >= 0;i--)
		{
			//��ñ���
			b = m.read(i,k) / m.read(k,k);
			//�б任
			for (j = 0;j < m.n;j++)
			{
				temp = m.read(i,j) - b * m.read(k,j);
				m.write(i,j,temp);
			}
		}
	}
	//����߷���Ϊ��λ����
	for (i = 0;i < m.m;i++)
	{
		if (m.read(i,i) != 1)
		{
			//��ñ���
			b = 1 / m.read(i,i);
			//�б任
			for (j = 0;j < m.n;j++)
			{
				temp = m.read(i,j) * b;
				m.write(i,j,temp);
			}
		}
	}
	//��������
	for (i = 0;i < B->m;i++)
	{
		for (j = 0;j < B->m;j++)
		{
			B->write(i,j,m.read(i,j + m.m));
		}
	}
	//�ͷ��������
	m.free_matrix();
	
	return 1;
}

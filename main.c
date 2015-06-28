#include <stdio.h>
#include <stdlib.h>
#define MAX_FILE_NAME 50

////////////////////CONST_DEF////////////////////
/*****la.h***CONST_DEF*****/
#define MAX_LINE 81
#define MAX_IDEN 80
#define MAX_STR 80
#define RESERVE_WORD_NUM 13
#include <string.h>
#include <ctype.h>
/*****end_of_la.h***CONST_DEF*****/

/*****sa.h***CONST_DEFINE*****/
#define MAX_NAME 80
#define MAX_OP 15
#define MAX_VALUE_LEN 80
#define MAX_TABLE 200
#define MAX_LABEL 200
#define MAX_LOCAL 200
#define MAX_SET 38
#define MAX_STACK 20
#define MAX_STR_TABLE 200

#include <string.h>
/*****end_of_sa.h***CONST_DEF*****/

/*****toasm.h***CONST_DEF*****/
#include <string.h>
#define MAX_ASSIGN 100
#define MAX_TMP_VAR 100
/*****end_of_toasm.h***CONST_DEF*****/

/*****optimize.h***CONST_DEF*****/
#define MAX_BLOCK 100
#define MAX_PROC 20
/*****end_of_optimize.h***CONST_DEF*****/
////////////////////CONST_DEF////////////////////

////////////////////GLOBAL_VAR////////////////////
/*****la.h***GLOBAL_VAR*****/
enum symbol {IDEN, INTCON, CHARCON, STRCON, CONSTTK, INTTK, CHARTK, VOIDTK,
				MAINTK, IFTK, ELSETK, DOTK, WHILETK, FORTK, SCANFTK, PRINTFTK,
				RETURNTK, PLUS, MINU, MULT, DIV, LSS, LEQ, GRE, GEQ, EQL,
				NEQ, ASSIGN, SEMICN, COMMA, QMARK, DQMARK, LPARENT, RPARENT,
				LBRACK, RBRACK, LBRACE, RBRACE};
enum symbol rwsym[] = {
    CHARTK, CONSTTK, DOTK, ELSETK, FORTK, IFTK, INTTK,
    MAINTK, PRINTFTK, RETURNTK, SCANFTK, VOIDTK, WHILETK
};
char * rword[] = {
    "char", "const", "do", "else", "for", "if", "int",
    "main", "printf", "return", "scanf", "void", "while"
};

int cc = 0; //当前行读入字符数
int ll = 0; //行长度
int line_num = 0;   //读入的总行数
int kk = 0; //当前单词的长度
int num = 0;    //最后读入的整数
char ch = ' ';    //当前读入的字符
char chcon = 0; //最后读入的字符常量
char strcon[MAX_STR];   //最后读入的字符串常量
char line[MAX_LINE] = {0};    //当前行
char iden[MAX_IDEN] = {0};    //最后读入的标识符
int type;       //最后读入的类型保留字（char为2，int为1）
int if_signed;  //最后读入的数字是否有符号，0无1有
int end_flag = 0;	//当前是否已经到达文件结尾，用在get_ch()中
enum symbol sym;

FILE *in, *out, *err;
/**********end_of_la.h***GLOBAL_VAR**********/

/*****sa.h***GLOBAL_VAR*****/
struct record
{
    char name[MAX_NAME];
    /*code - kind
      0    - nul
      1    - const
      2    - var
      3    - func
      4    - array
      5    - para
      6    - tmp*/
    int kind;
    /*code - type
      0    - nul
      1    - int
      2    - char
      3    - str*/
    int type;
    union
    {
        int int_value;
        char char_value;
        char str_value[MAX_VALUE_LEN];
    };
    //int addr;
} table[MAX_TABLE];

enum quat_op {
    ADD, SUB, MUL, DIVV, NEG, MOV,
    JZ, JNZ, JL, JLE, JG, JGE, JE,
    JNE, RET, WRITE, READ, PARAV,
    CALL, JMP, NOP, MAINF, EMAINF,
    FUNC, EFUNC, CONST, VAR
};

const char * quat_op_name[] = {
    "ADD", "SUB", "MUL", "DIVV", "NEG", "MOV",
    "JZ", "JNZ", "JL", "JLE", "JG", "JGE", "JE",
    "JNE", "RET", "WRITE", "READ", "PARAV",
    "CALL", "JMP", "NOP", "MAINF", "EMAINF",
    "FUNC", "EFUNC", "CONST", "VAR"
};

struct quaternion
{
    enum quat_op op;
    char operand_a[MAX_OP];
    char operand_b[MAX_OP];
    char operand_r[MAX_OP];
    int label;  //需要初始化为-1
} quat_table[MAX_TABLE];

char label_table[MAX_LABEL][MAX_NAME];
char str_table[MAX_STR_TABLE][MAX_STR + 1];

int lvx = 0;    //局部变量在符号表中的起始位置
int lx = 0;     //参数在符号表中的起始位置
int tx = 0;     //符号表当前指针
int tmpx = 0;   //临时变量分配计数器
int lbx = 0;    //label表指针
int qtx = 0;    //四元式表指针
int strx = 0;   //字符串常量表指针
int is_gl = 0;  //当前在局部还是全局的标志，0全局1局部
int return_flag = 0;	//函数是否包含合法的返回语句
int error_flag = 0;		//是否出错
/*****end_of_sa.h***GLOBAL_VAR*****/

/*****toasm.h***GLOBAL_VAR*****/
struct addr_assign
{
    char name[MAX_NAME];
    int tpye;
    int offset;
} assign_table[MAX_ASSIGN];

int addrx = 0;      //变量地址分配表指针
int addrtx = 0;     //临时变量起始分配地址
int esp = 0;        //堆栈分配指针，esp并非真正的esp，而是esp+4
int qq = 0;     //四元式遍历指针
//int fi = 0;      //当前分析的函数在符号表中的位置
FILE *asm_out;
/*****end_of_toasm.h***GLOBAL_VAR*****/

/*****optimize.h***GLOBAL_VAR*****/
int blocks[MAX_PROC][MAX_BLOCK];
int blx = 0;    //基本块表外层指针
struct quaternion opt_quat[MAX_TABLE];  //优化后的四元式，label需要初始化为-1
int oqtx = 0;   //优化四元式表指针

struct dag_node
{
    char op[MAX_OP];
    int lchild;
    int rchild;
} dag_table[MAX_TABLE]; //DAG图
int dtx = 0;    //DAG图表指针
struct node_record
{
    char name[MAX_OP];
    int num;
} node_table[MAX_TABLE];    //DAG节点表
int ntx = 0;    //DAG图节点表指针
/*****end_of_optimize.h***GLOBAL_VAR*****/
////////////////////GLOBAL_VAR////////////////////

////////////////////FUNC_DEF////////////////////
/*****la.h***FUNC_DEF*****/
void get_ch();
void getsym();
void error(int);
/*****end_of_la.h***FUNC_DEF*****/

/*****sa.h***FUNC_DEF*****/
void insert_table(char *name, int kind, int type, int int_value, char char_value, char *str_value);
void emit(enum quat_op op, char *a, char *b, char *r);
int gen_label();
int gen_tmp_var(char *name, int type);
int insert_str();
int position(char *name);
void print_quat();
void program();
void const_definition();
void const_declaration();
void var_definition();
void var_declaration();
void rfunc_definition();
void nfunc_definition();
void paras_definition();
void main_func();
void compound_statement();
void statements();
void statement();
void if_statement();
void while_statement();
void for_statement();
int invoke_func(int i);
void paras_value(int i);
void assign_statement(int i);
void read_statement();
void write_statement();
void return_statement();
int condition(int n);
int expression(int *flag);
int term(int *flag, int *index, int *index_type);
int factor(int *flag, int *index, int *index_type);
int gen_op(char *op, int flag, int index, int index_type, int i);
void int_num();
/*****end_of_sa.h***FUNC_DEF*****/

/*****toasm.h***FUNC_DEF*****/
void toasm();
void insert_assign_table(char *name, int length, int type);
int search_addr(char *name);
int search_array_addr(const char *name, int *index_off, int *index_type);
int find_type(char *name);
void asm_head();
void asm_data();
void asm_code();
void asm_array(char *name, int offset, int index, int index_type);
void asm_add();
void asm_sub();
void asm_mul();
void asm_divv();
void asm_neg();
void asm_mov();
void asm_jcdt(enum quat_op op);
void asm_ret();
void asm_write();
void asm_read();
void asm_parav();
void asm_call();
void asm_jump();
void asm_nop();
/*****end_of_toasm.h***FUNC_DEF*****/

/*****optimize.h***FUNC_DEF*****/
int proc_block(int i);
void divide_block();
void copy_oquat();
void opt_dag();
void write_dag();
void write_opt_quat(int node);
void insert_opt_quat_table(enum quat_op op, char *op_a, char *op_b, char *op_r);
int has_parent(int node, int last, int mid_nodes[]);
int select_mid_nodes(int mid_nodes[]);
void build_dag(const int s, const int e);
int test_array(const char *name, char *op, char *index);
void gen_node_op(const char *name, enum quat_op op, char *node_name);
int search_node_table(char *name);
int search_dag_table(char *op, int lchild, int rchild);
int insert_dag_table(char *op, int lchild, int rchild);
void insert_node_table(char *name, int num);
void opt_local();
void parse_line(const char *line, char *a, char *b);
/*****end_of_optimize.h***FUNC_DEF*****/
////////////////////FUNC_DEF////////////////////

////////////////////CODE////////////////////
/*****error.c*****/
void error(int code)
{
    switch(code)
    {
        case 0:
            fprintf(err, "在第%d行出错：字符常量的字符非法\n", line_num);
            break;
        case 1:
            fprintf(err, "在第%d行出错：字符常量缺少\'\n", line_num);
            break;
        case 2:
            fprintf(err, "在第%d行出错：字符串常量过长\n", line_num);
            break;
        case 3:
            fprintf(err, "在第%d行出错：非法Token\n", line_num);
            break;
        case 4:
            fprintf(err, "在第%d行出错：=左右类型不匹配，如果是赋值语句，可能是将int赋值给char\n", line_num);
            break;
        case 5:
            fprintf(err, "在第%d行出错：标识符或变量后应为=\n", line_num);
            break;
        case 6:
            fprintf(err, "在第%d行出错：int,char后应为标识符\n", line_num);
            break;
        case 7:
            fprintf(err, "在第%d行出错：const后应为int,char\n", line_num);
            break;
        case 8:
            fprintf(err, "在第%d行出错：缺少;\n", line_num - 1);
            break;
        case 9:
            fprintf(err, "在第%d行出错：0不应该有符号\n", line_num);
            break;
        case 10:
            fprintf(err, "在第%d行出错：不应该有前0\n", line_num);
            break;
        case 11:
            fprintf(err, "在第%d行出错：缺少)\n", line_num);
            break;
        case 12:
            fprintf(err, "在第%d行出错：缺少]\n", line_num);
            break;
        case 13:
            fprintf(err, "在第%d行出错：缺少}\n", line_num);
            break;
        case 14:
            fprintf(err, "在第%d行出错：数组长度应为无符号整数\n", line_num);
            break;
        case 15:
            fprintf(err, "在第%d行出错：main函数后面有多余的字符或定义\n", line_num);
            break;
        case 16:
            fprintf(err, "在第%d行出错：参数表定义错误，缺少类型标识符\n", line_num);
            break;
        case 17:
            fprintf(err, "在第%d行出错：函数头定义错误\n", line_num);
            break;
        case 18:
            fprintf(err, "在第%d行出错：函数体定义错误，缺少{\n", line_num);
            break;
        case 19:
            fprintf(err, "在第%d行出错：条件错误，缺少(\n", line_num);
            break;
        case 20:
            fprintf(err, "在第%d行出错：缺少while\n", line_num);
            break;
        case 21:
            fprintf(err, "在第%d行出错：for条件语法错误\n", line_num);
            break;
        case 22:
            fprintf(err, "在第%d行出错：不期望的标识符\n", line_num);
            break;
        case 23:
            fprintf(err, "在第%d行出错：读语句、写语句或返回语句格式错误，缺少(\n", line_num);
            break;
        case 24:
            fprintf(err, "在第%d行出错：读语句或写语句格式错误，,后应为标识符\n", line_num);
            break;
        case 25:
            fprintf(err, "在第%d行出错：不可识别的语句\n", line_num);
            break;
        case 26:
            fprintf(err, "在第%d行出错：未定义的标识符\n", line_num);
            break;
        case 27:
            fprintf(err, "在第%d行出错：数组引用越界\n", line_num);
            break;
        case 28:
            fprintf(err, "在第%d行出错：不期望的标识符类型\n", line_num);
            break;
        case 29:
            fprintf(err, "在第%d行出错：函数调用的参数个数或类型不匹配\n", line_num);
            break;
        case 30:
            fprintf(err, "在第%d行出错：重复定义的标识符\n", line_num);
            break;
		case 31:
            fprintf(err, "在第%d行出错：源文件不完整\n", line_num);
            break;
		case 32:
            fprintf(err, "在第%d行出错：符号后应为数字\n", line_num);
            break;
		case 33:
            fprintf(err, "在第%d行出错：无返回值函数不能作为因子\n", line_num);
            break;
		case 34:
            fprintf(err, "在第%d行出错：有返回值的函数没有合法的返回语句\n", line_num);
            break;
    }
	error_flag = 1;
}
/*****end of error.c*****/

/*****la.c*****/
void get_ch()
{
    if(cc >= ll)
    {
        if(!fgets(line, MAX_LINE-1, in))
        {
			if(!end_flag)
			{
				end_flag = 1;
				ch = ' ';
				return ;
			}
			else
			{
				error(31);
				printf("编译失败！详细错误请查看err.txt文件！");
				exit(0);
			}
        }
        ll = strlen(line);
        ll--;
        cc = 0;
        line_num++;
    }
    ch = line[cc];
    cc++;
}

void getsym()
{
    int k = 0;
    int i, j, r;
    while(isspace(ch))
        get_ch();
    if(isalpha(ch) || ch == '_')
    {
        k = 0;
        do
        {
            iden[k++] = ch;
            get_ch();
        } while(isalpha(ch) || isdigit(ch) || ch == '_');
        iden[k] = '\0';
        if(k > kk)
            kk = k;
        else
            while(kk > k)
                iden[--kk] = '\0';
        i = 0;
        j = RESERVE_WORD_NUM - 1;
        while(i <= j)
        {
            k = (i + j) / 2;
            if(!(r = strcmp(iden, rword[k])))
            {
                sym = rwsym[k];
                if(!k)
                    type = 2;
                else if(k == 6)
                    type = 1;
                break;
            }
            else if(r > 0)
                i = k + 1;
            else
                j = k - 1;
        }
        if(r)
            sym = IDEN;
    }
    else if(isdigit(ch))
    {
        num = 0;
        if(ch == '0')
        {
            get_ch();
            if(isdigit(ch))
                error(10);
        }
		while(isdigit(ch))
        {
            num = ch - '0' + 10 * num;
            get_ch();
        }
        sym = INTCON;
    }
    else if(ch == '\'')
    {
        sym = QMARK;
        get_ch();
        chcon = ch;
        if(!isalpha(ch) && !isdigit(ch) && ch != '+' && ch != '-' && ch != '*' && ch != '/')
        {
            error(0);
            chcon = '\0';
        }
        get_ch();
        if(ch != '\'')
            error(1);
        sym = CHARCON;
        get_ch();
    }
    else if(ch == '\"')
    {
        k = 0;
        strcon[0] = '\0';
        sym = DQMARK;
        get_ch();
        while(ch != '\"')
        {
            strcon[k++] = ch;
            if(k == MAX_STR - 1)
                error(2);
            get_ch();
        }
        strcon[k] = '\0';
        //TO DO: 一直到文件尾都没出现右引号
        sym = STRCON;
        get_ch();
    }
    else if(ch == '+')
    {
        sym = PLUS;
		get_ch();
    }
    else if(ch == '-')
    {
        sym = MINU;
		get_ch();
    }
    else if(ch == '*')
    {
        sym = MULT;
        get_ch();
    }
    else if(ch == '/')
    {
        sym = DIV;
        get_ch();
    }
    else if(ch == '<')
    {
        get_ch();
        if(ch == '=')
        {
            sym = LEQ;
            get_ch();
        }
        else{
            sym = LSS;
        }
    }
    else if(ch == '>')
    {
        get_ch();
        if(ch == '=')
        {
            sym = GEQ;
            get_ch();
        }
        else{
            sym = GRE;
        }
    }
    else if(ch == '=')
    {
        get_ch();
        if(ch == '=')
        {
            sym = EQL;
            get_ch();
        }
        else{
            sym = ASSIGN;
        }
    }
    else if(ch == '!')
    {
        get_ch();
        if(ch == '=')
        {
            sym = NEQ;
            get_ch();
        }
        else
            error(3);
    }
    else
    {
        switch(ch)
        {
            case ';':
                sym = SEMICN;
                break;
            case ',':
                sym = COMMA;
                break;
            case '(':
                sym = LPARENT;
                break;
            case ')':
                sym = RPARENT;
                break;
            case '[':
                sym = LBRACK;
                break;
            case ']':
                sym = RBRACK;
                break;
            case '{':
                sym = LBRACE;
                break;
            case '}':
                sym = RBRACE;
                break;
            default:
                error(3);
        }
        get_ch();
    }
}
/*****end of la.c*****/

/*****sa.c*****/
void insert_table(char *name, int kind, int type, int int_value, char char_value, char *str_value)
{
    strcpy(table[tx].name, name);
    table[tx].kind = kind;
    table[tx].type = type;
    if(kind == 1)
    {
        switch(type)
        {
            case 1: table[tx].int_value = int_value; break;
            case 2: table[tx].char_value = char_value; break;
            case 3: strcpy(table[tx].str_value, str_value); break;
        }
    }
    else if(kind == 4)
        table[tx].int_value = int_value;
    else
        table[tx].int_value = 0;
    tx++;
}

void emit(enum quat_op op, char *a, char *b, char *r)
{
    quat_table[qtx].op = op;
    strcpy(quat_table[qtx].operand_a, a);
    strcpy(quat_table[qtx].operand_b, b);
    strcpy(quat_table[qtx].operand_r, r);
    qtx++;
}

int gen_label()
{
    sprintf(label_table[lbx], "label_%d", lbx);
    lbx++;
    return lbx - 1;
}

int gen_tmp_var(char *name, int type)
{
    sprintf(name, "$%d", tmpx);
    insert_table(name, 6, type, 0, 0, NULL);
    tmpx++;
    return tx - 1;
}

int insert_str()
{
    strcpy(str_table[strx], strcon);
    strx++;
    return strx - 1;
}

int position(char *name)
{
    int p;
    for(p = tx; p >= 0; p--)
        if(!strcmp(name, table[p].name))
            return p;
    return p;
}

void print_quat()
{
    int i;
    for(i = 0; i < qtx; i++)
    {
        if(quat_table[i].label != -1)
            fprintf(out, "%s:\n", label_table[quat_table[i].label]);
        fprintf(out, "%10s %10s %10s %10s\n",
                quat_op_name[quat_table[i].op], quat_table[i].operand_a, quat_table[i].operand_b, quat_table[i].operand_r);
    }
}

//*这里语法分析出错后跳到下一个;继续
void program()
{
    if(sym == CONSTTK)
    {
        getsym();
        const_declaration();
    }
    if(sym == INTTK || sym == CHARTK)
    {
        var_declaration();
    }
    if(sym == LPARENT)
    {
        rfunc_definition();
    }
    while(sym == INTTK || sym == CHARTK || sym == VOIDTK)
    {
        if(sym == VOIDTK)
        {
            getsym();
            nfunc_definition();
        }
        else
        {
            getsym();
            rfunc_definition();
        }
    }
    main_func();
}

void const_definition()
{
    int i;
    char tmp[MAX_OP];
    do
    {
        if(sym == IDEN)
        {
            i = position(iden);
            if((is_gl && i >= lx) || (!is_gl && i >= 0))
                error(30);
            getsym();
            if(sym == ASSIGN)
            {
                getsym();
				int_num();
                if(sym == INTCON && type == 1)
                {
                    sprintf(tmp, "%d", num);
                    insert_table(iden, 1, 1, num, 0, NULL);
					if(!is_gl)
						emit(CONST, "INT", iden, tmp);
					else
						emit(CONST, "INT", "", tmp);
                }
                else if(sym == CHARCON && type == 2)
                {
                    sprintf(tmp, "%d", chcon);
                    insert_table(iden, 1, 2, 0, chcon, NULL);
					if(!is_gl)
						emit(CONST, "CHAR", iden, tmp);
					else
						emit(CONST, "CHAR", "", tmp);
                }
                else
                    error(4);
            }
            else
			{
                error(5);
				while(sym != COMMA && sym != SEMICN)
					getsym();
			}
        }
        else
		{
            error(6);
			while(sym != COMMA && sym != SEMICN)
				getsym();
		}
        getsym();
    } while(sym == COMMA);

}

void const_declaration()
{
    do
    {
		if(sym == CONSTTK)
			getsym();
        if(sym == INTTK || sym == CHARTK)
        {
            getsym();
            const_definition();
        }
        else
		{
            error(7);
			while(sym != SEMICN)
				getsym();
		}
        if(sym == SEMICN)
            getsym();
        else
            error(8);
    } while(sym == CONSTTK);
}

void var_definition()
{
    int i;
    char tmp[MAX_OP];
    do
    {
        if(sym == COMMA)
            getsym();
        if(sym == INTTK || sym == CHARTK)
            getsym();
        if(sym == IDEN)
        {
            i = position(iden);
            if((is_gl && i >= lx) || (!is_gl && i >= 0))
                error(30);
            getsym();
            if(sym == LBRACK)
            {
                getsym();
				int_num();
                if(sym == INTCON && if_signed == 0)
                {
                    sprintf(tmp, "%d", num);
                    getsym();
                    if(sym == RBRACK)
                    {
                        insert_table(iden, 4, type, num, 0, NULL);
                        if(type == 1)
                            emit(VAR, "INT", "", tmp);
                        else
                            emit(VAR, "CHAR", "", tmp);
						if(!is_gl)
							strcpy(quat_table[qtx-1].operand_b, iden);
                        getsym();
                    }
                    else
                        error(12);
                }
                else
                    error(14);
            }
            else if(sym != LPARENT)
            {
                insert_table(iden, 2, type, 0, 0, NULL);
                if(type == 1)
                    emit(VAR, "INT", "", "");
                else
                    emit(VAR, "CHAR", "", "");
				if(!is_gl)
					strcpy(quat_table[qtx-1].operand_b, iden);
            }
        }
        else
		{
			error(6);
			while(sym != COMMA && sym != SEMICN && sym != LBRACE)
				getsym();
		}
    } while(sym == COMMA);
}

void var_declaration()
{
    do
    {
        var_definition();
        if(sym == LPARENT)
            break;
        if(sym == SEMICN)
            getsym();
        else if(sym == LBRACE)
		{
			getsym();
			compound_statement();
		}
		else
            error(8);
    } while(sym == INTTK || sym == CHARTK);
}

void rfunc_definition()
{
    int i;
    if(sym == IDEN)//....
    {
        i = position(iden);
        if(i >= 0)
            error(30);
        getsym();
    }
    else if(sym != LPARENT)
	{
        error(17);
		while(sym != LBRACE && sym != SEMICN)
			getsym();
		if(sym == LBRACE)
		{
			i = -1;
			goto label_1;
		}
		else
		{
			is_gl = 1;
            getsym();
			return_flag = 0;
			goto label_2;
		}
	}
    if(sym == LPARENT)
    {
        insert_table(iden, 3, type, 0, 0, NULL);
        i = tx - 1;
        lx = tx;
        getsym();
        paras_definition();
        if(sym == RPARENT)
            getsym();
        else
            error(11);
label_1:
        if(sym == LBRACE)
        {
            is_gl = 1;
			if(i >= 0)
				emit(FUNC, table[i].name, "", "");
            getsym();
			return_flag = 0;
label_2:
            compound_statement();
			if(return_flag != 1)
				error(34);
            emit(EFUNC, table[i].name, "", "");
            is_gl = 0;
            //清除符号表中参数名...不用了，因为参数和局部变量均有lx和lvx指出作用域...还是要，函数和参数重名
            for(i = lx; i < tx; i++)
                table[i].name[0] = '\0';
			tx = lvx;
            if(sym == RBRACE)
                getsym();
            else
                error(13);
        }
        else
            error(18);
    }
    else
        error(17);
}

void nfunc_definition()
{
    int i;
    if(sym == IDEN)
    {
        i = position(iden);
        if(i >= 0)
            error(30);
        getsym();
    }
    else if(sym == MAINTK)
        return ;
    else
	{
        error(6);
		while(sym != LBRACE && sym != SEMICN)
			getsym();
		if(sym == LBRACE)
			goto label_3;
		else
		{
			is_gl = 1;
            getsym();
			return_flag = 0;
			goto label_4;
		}
	}
    if(sym == LPARENT)
    {
        insert_table(iden, 3, 0, 0, 0, NULL);
        i = tx - 1;
        lx = tx;
        getsym();
        paras_definition();
        if(sym == RPARENT)
            getsym();
        else
            error(11);
label_3:
        if(sym == LBRACE)
        {
            is_gl = 1;
            emit(FUNC, table[i].name, "", "");
            getsym();
			return_flag = 0;
label_4:
            compound_statement();
			if(return_flag)
				error(34);
            emit(EFUNC, table[i].name, "", "");
            is_gl = 0;
            tx = lvx;
            //清除符号表中参数名...不用了，因为参数和局部变量均有lx和lvx指出作用域...还是要，函数和参数重名
            for(i = lx; i < lvx; i++)
                table[i].name[0] = '\0';
            if(sym == RBRACE)
                getsym();
            else
                error(13);
        }
        else
            error(18);
    }
    else
	{
        error(17);
		while(sym != LBRACE && sym != SEMICN)
			getsym();
		if(sym == LBRACE)
			goto label_3;
		else
		{
			is_gl = 1;
            getsym();
			return_flag = 0;
			goto label_4;
		}
	}
}

void paras_definition()
{
    int i;
    if(sym == RPARENT)
    {
        lvx = tx;
        return ;
    }
	else if(sym == LBRACE)
		return ;
    else
    {
        do
        {
            if(sym == COMMA)
                getsym();
            if(sym == INTTK || sym == CHARTK)
            {
                getsym();
                if(sym == IDEN)
                {
                    i = position(iden);
                    if(i >= lx)
                        error(30);
                    insert_table(iden, 5, type, 0, 0, NULL);
                    getsym();
                }
                else
				{
                    error(6);
					while(sym != COMMA && sym != RPARENT && sym != LBRACE)
						getsym();
				}
            }
            else
			{
                error(16);
				while(sym != COMMA && sym != RPARENT && sym != LBRACE)
					getsym();
			}
        } while(sym == COMMA);
        lvx = tx;
    }
}

void main_func()
{
    if(sym == VOIDTK)
        getsym();
    else if(sym != MAINTK)
        error(17);
    if(sym == MAINTK)
    {
        getsym();
        if(sym == LPARENT)
        {
            lx = tx;
            getsym();
            if(sym == RPARENT)
            {
                lvx = tx;
                getsym();
label_5:
                if(sym == LBRACE)
                {
                    is_gl = 1;
                    emit(MAINF, "", "", "");
                    getsym();
					return_flag = 0;
label_6:
                    compound_statement();
					if(return_flag)
						error(34);
                    emit(EMAINF, "", "", "");
                    is_gl = 0;
                    tx = lvx;
                    if(sym == RBRACE)
					{
                        ;//到达结尾，如果还有字符怎么报错？
						if(!end_flag)
						{
							while(isspace(ch))
							{
								get_ch();
								if(end_flag)
									return ;
							}
							error(15);
						}
					}
                    else
                        error(13);
                }
                else
                    error(17);
            }
            else
                error(11);
        }
        else
		{
            error(17);
			while(sym != LBRACE && sym != SEMICN)
				getsym();
			if(sym == LBRACE)
				goto label_5;
			else
			{
				is_gl = 1;
				getsym();
				return_flag = 0;
				goto label_6;
			}
		}
    }
    else
        error(17);
}

void compound_statement()
{
    if(sym == CONSTTK)
    {
        getsym();
        const_declaration();
    }
    if(sym == INTTK || sym == CHARTK)
        var_declaration();
    statements();
}

void statements()
{
    while(sym == IFTK || sym == DOTK || sym == FORTK || sym == LBRACE ||
          sym == IDEN || sym == SCANFTK || sym == PRINTFTK || sym == RETURNTK || sym == SEMICN)
        statement();
}

void statement()
{
    int i;
    if(sym == IFTK)
    {
        getsym();
        if_statement();
    }
    else if(sym == DOTK)
    {
        getsym();
        while_statement();
    }
    else if(sym == FORTK)
    {
        getsym();
        for_statement();
    }
    else if(sym == LBRACE)
    {
        getsym();
        statements();
        if(sym == RBRACE)
            getsym();
        else
            error(13);
    }
    else if(sym == IDEN)
    {
        i = position(iden);
        if(i < 0)
		{
            error(26);
			while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
				getsym();
			if(sym == SEMICN)
				getsym();
		}
        getsym();
        if(sym == LPARENT)
        {
            getsym();
            invoke_func(i);
            if(sym == SEMICN)
                getsym();
            else
                error(8);
        }
        else if(sym == LBRACK || sym == ASSIGN)
        {
            assign_statement(i);
            if(sym == SEMICN)
                getsym();
            else
                error(8);
        }
        else
		{
            error(22);
			while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
				getsym();
			if(sym == SEMICN)
				getsym();
		}
    }
    else if(sym == SCANFTK)
    {
        getsym();
        read_statement();
        if(sym == SEMICN)
            getsym();
        else
            error(8);
    }
    else if(sym == PRINTFTK)
    {
        getsym();
        write_statement();
        if(sym == SEMICN)
            getsym();
        else
            error(8);
    }
    else if(sym == RETURNTK)
    {
        getsym();
        return_statement();
        if(sym == SEMICN)
            getsym();
        else
            error(8);
    }
    else if(sym == SEMICN)
    {
        emit(NOP, "", "", "");
        getsym();
    }
    else
	{
        error(25);
		while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
			getsym();
		if(sym == SEMICN)
			getsym();
	}
}

void if_statement()
{
    int label1 = -1, label2 = -1;
    int cx;
    if(sym == LPARENT)
    {
        getsym();
        cx = condition(0);
        label1 = gen_label();
        strcpy(quat_table[cx].operand_r, label_table[label1]);
        if(sym == RPARENT)
        {
            getsym();
            statement();
            if(sym == ELSETK)
            {
                label2 = gen_label();
                emit(JMP, "", "", label_table[label2]);
                getsym();
                quat_table[qtx].label = label1;
                statement();
                quat_table[qtx].label = label2;
            }
            else
                quat_table[qtx].label = label1;
        }
        else
            error(11);
    }
    else
        error(19);
}

void while_statement()
{
    int label, cx;
    label = gen_label();
    quat_table[qtx].label = label;
    statement();
    if(sym == WHILETK)
    {
        getsym();
        if(sym == LPARENT)
        {
            getsym();
            cx = condition(1);
            strcpy(quat_table[cx].operand_r, label_table[label]);
            if(sym == RPARENT)
                getsym();
            else
                error(11);
        }
        else
		{
            error(19);
			while(!(sym == IFTK || sym == DOTK || sym == FORTK || sym == LBRACE || sym == RBRACE ||
				sym == IDEN || sym == SCANFTK || sym == PRINTFTK || sym == RETURNTK || sym == SEMICN))
				getsym();
		}
    }
    else
        error(20);
}

void for_statement()
{
    int i, flag, cx, label1 = -1, label2 = -1;
    char op1[MAX_OP], op2[MAX_OP], tmp[MAX_OP];
    int incr = 0, error_flag = 0;
    enum quat_op incr_op = ADD;
    if(sym == LPARENT)
    {
        getsym();
        if(sym == IDEN)
        {
            i = position(iden);
            if(i < 0)
			{
                error(26);
				while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
					getsym();
				if(sym == SEMICN)
					getsym();
			}
            if(table[i].kind != 2 && table[i].kind != 5)
                error(28);
            gen_op(op1, 0, 0, 0, i);
            getsym();
            if(sym == ASSIGN)
            {
                getsym();
                i = expression(&flag);
                gen_op(op2, flag, 0, 0, i);
                emit(MOV, op2, "", op1);
                if(sym == SEMICN)
                {
                    getsym();
                    label1 = gen_label();
                    quat_table[qtx].label = label1;
                    cx = condition(0);
                    label2 = gen_label();
					strcpy(quat_table[cx].operand_r, label_table[label2]);
                    if(sym == SEMICN)
                    {
                        getsym();
                        if(sym == IDEN)
                        {
                            i = position(iden);
                            if(i < 0)
							{
                                error(26);
								while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
									getsym();
								if(sym == SEMICN)
									getsym();
							}
                            if(table[i].kind != 2 && table[i].kind != 5)
                                error(28);
                            gen_op(op1, 0, 0, 0, i);
                            getsym();
                            if(sym == ASSIGN)
                            {
                                getsym();
                                if(sym == IDEN)
                                {
                                    i = position(iden);
                                    if(i < 0)
									{
                                        error(26);
										while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
											getsym();
										if(sym == SEMICN)
											getsym();
									}
                                    if(table[i].kind != 2 && table[i].kind != 5)
                                        error(28);
                                    gen_op(op2, 0, 0, 0, i);
                                    getsym();
                                    if(sym == PLUS || sym == MINU)
                                    {
                                        incr_op = (sym == PLUS) ? ADD : SUB;
                                        getsym();
										int_num();
                                        if(sym == INTCON && !if_signed)
                                        {
                                            incr = num;
                                            getsym();
                                            if(sym == RPARENT)
                                            {
                                                getsym();
label_7:
                                                statement();
                                                sprintf(tmp, "%d", incr);
                                                emit(incr_op, op2, tmp, op1);
                                                emit(JMP, "", "", label_table[label1]);
                                                quat_table[qtx].label = label2;
                                            }
                                            else
                                                error(11);
                                        }
                                        else
										{error(21);error_flag = 1;}
                                    }
                                    else
                                    {error(21);error_flag = 1;}
                                }
                                else
                                {error(21);error_flag = 1;}
                            }
                            else
                            {error(21);error_flag = 1;}
                        }
                        else
                        {error(21);error_flag = 1;}
                    }
                    else
                    {error(21);error_flag = 1;}
                }
                else
                {error(21);error_flag = 1;}
            }
            else
            {error(21);error_flag = 1;}
        }
        else
        {error(21);error_flag = 1;}
    }
    else
        error(19);
	if(error_flag)
	{
		while(sym != RPARENT && sym != LBRACE)
			getsym();
		getsym();
		error_flag = 0;
		goto label_7;
	}
}

int invoke_func(int i)
{
    int iproc = i;
    if(table[i].kind != 3)
        error(28);
    paras_value(i);
    emit(CALL, table[iproc].name, "", "");
    if(sym == RPARENT)
    {
        getsym();
    }
    else
        error(11);
    return table[iproc].type;
}

void paras_value(int i) //i为函数声明在符号表中位置
{
    int j, flag, k = i + 1;
    char op[MAX_OP];
	int r = 0;
	char op_STACK[MAX_STACK][MAX_OP];
    if(sym == RPARENT)
    {
        if(table[i+1].kind == 5)
            error(29);
        return ;
    }
    do
    {
        if(sym == COMMA)
            getsym();
        if(k == tx || table[k].kind != 5)
		{
            error(29);
			while(sym != RPARENT && sym != SEMICN)
				getsym();
			return ;
		}
        j = expression(&flag);
        if((flag <= 1 && table[j].type >= table[k].type) ||
           (flag >= 2 && flag - 1 >= table[k].type))
        {
            gen_op(op, flag, 0, 0, j);
			strcpy(op_STACK[r++], op);
            //emit(PARAV, op, "", "");
        }
        else
            error(29);
        k++;
    } while(sym == COMMA);
	if(table[k].kind == 5)
		error(29);
	for(r--; r >= 0; r--)
		emit(PARAV, op_STACK[r], "", "");
}

void assign_statement(int i)    //i为左操作数在符号表中的位置
{
    int j, flag;
    char op1[MAX_OP], op2[MAX_OP];
	int type1, type2;
    if(table[i].kind != 2 && table[i].kind != 4 && table[i].kind != 5)
        error(28);
    if(sym == LBRACK)
    {
        if(table[i].kind != 4)
            error(28);
        getsym();
        j = expression(&flag);
        type1 = gen_op(op1, 1, j, flag, i);
        if(sym == RBRACK)
            getsym();
        else
            error(12);
    }
    else
	{
		if(table[i].kind != 2 && table[i].kind != 5)
			error(28);
		else
			type1 = gen_op(op1, 0, 0, 0, i);
	}
    if(sym == ASSIGN)
    {
        getsym();
        j = expression(&flag);
        type2 = gen_op(op2, flag, 0, 0, j);
		if(type1 > type2)
			error(4);
		else
			emit(MOV, op2, "", op1);
    }
    else
	{
        error(5);
		while(sym != SEMICN)
			getsym();
	}
}

void read_statement()
{
    int i;
    char op[MAX_OP];
    if(sym == LPARENT)
    {
        do
        {
            getsym();
            if(sym == IDEN)
            {
                i = position(iden);
                if(i < 0)
				{
                    error(26);
					while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
						getsym();
					if(sym == SEMICN)
						getsym();
				}
                if(table[i].kind != 2 && table[i].kind != 5)
                    error(28);
                gen_op(op, 0, 0, 0, i);
                emit(READ, "", "", op);
                getsym();
            }
            else
			{
                error(24);
				while(sym != COMMA && sym != RPARENT)
					getsym();
			}
        } while(sym == COMMA);
        if(sym == RPARENT)
            getsym();
        else
            error(11);
    }
    else
	{
        error(23);
		while(!(sym == IFTK || sym == DOTK || sym == FORTK || sym == LBRACE || sym == RBRACE ||
			sym == IDEN || sym == SCANFTK || sym == PRINTFTK || sym == RETURNTK || sym == SEMICN))
			getsym();
	}
}

void write_statement()
{
    int i, flag, str_index;
    char op[MAX_OP];
    char str[MAX_OP];
    if(sym == LPARENT)
    {
        getsym();
        if(sym == STRCON)
        {
            str_index = insert_str();
            sprintf(str, "__str%d", str_index);
            getsym();
            if(sym == COMMA)
            {
                getsym();
                i = expression(&flag);
                gen_op(op, flag, 0, 0, i);
                emit(WRITE, str, op, "");
            }
            else
                emit(WRITE, str, "", "");
        }
        else
        {
            i = expression(&flag);
            gen_op(op, flag, 0, 0, i);
            emit(WRITE, "", op, "");
        }
        if(sym == RPARENT)
            getsym();
        else
            error(11);
    }
    else
	{
        error(23);
		while(!(sym == IFTK || sym == DOTK || sym == FORTK || sym == LBRACE || sym == RBRACE ||
			sym == IDEN || sym == SCANFTK || sym == PRINTFTK || sym == RETURNTK || sym == SEMICN))
			getsym();
	}
}

void return_statement()
{
    int i, flag;
    char op[MAX_OP];
    if(sym == LPARENT)
    {
		return_flag = 1;
        getsym();
        i = expression(&flag);
        gen_op(op, flag, 0, 0, i);
        emit(RET, op, "", "");
        if(sym == RPARENT)
            getsym();
        else
            error(11);
    }
    else
	{
        emit(RET, "", "", "");
		if(sym != SEMICN)
			error(23);
		while(sym != SEMICN)
			getsym();
	}
}

int condition(int n) //返回待填入目的操作数（跳转label）的四元式编号，n = 0不符合条件跳转，n = 1符合条件跳转
{
    int flag, i;
    char op_a[MAX_OP], op_b[MAX_OP];
	enum symbol op_sym;
    i = expression(&flag);
    gen_op(op_a, flag, 0, 0, i);
	if(sym == LSS || sym == LEQ || sym == GRE || sym == GEQ || sym == EQL || sym == NEQ)
		op_sym = sym;
	else
	{
		if(n)
            emit(JNZ, op_a, "", "");
        else
            emit(JZ, op_a, "", "");
        return qtx - 1;
	}
	getsym();
	i = expression(&flag);
    gen_op(op_b, flag, 0, 0, i);
    switch(op_sym)
    {
        case LSS:
            if(n)
                emit(JL, op_a, op_b, "");
            else
                emit(JGE, op_a, op_b, "");
            break;
        case LEQ:
            if(n)
                emit(JLE, op_a, op_b, "");
            else
                emit(JG, op_a, op_b, "");
            break;
        case GRE:
            if(n)
                emit(JG, op_a, op_b, "");
            else
                emit(JLE, op_a, op_b, "");
            break;
        case GEQ:
            if(n)
                emit(JGE, op_a, op_b, "");
            else
                emit(JL, op_a, op_b, "");
            break;
        case EQL:
            if(n)
                emit(JE, op_a, op_b, "");
            else
                emit(JNE, op_a, op_b, "");
            break;
        case NEQ:
            if(n)
                emit(JNE, op_a, op_b, "");
            else
                emit(JE, op_a, op_b, "");
            break;
    }
    return qtx - 1;
}

int expression(int *flag)   //此处flag不能为1
{
    int rs = 0, index, index_type;
    int i, type1, type2;
	int caflag = 0;
    enum quat_op op;
    char op_a[MAX_OP] = {0};
    char op_b[MAX_OP] = {0};
	char tmp[MAX_OP] = {0};
    if(sym == PLUS || sym == MINU)
    {
        rs = (sym == PLUS) ? 0 : 1;
        getsym();
    }
    i = term(flag, &index, &index_type);
    if(*flag == 1)  //若表达式是数组引用，则将值取出存入临时变量，以解决a[a[a[2]]]的情况
    {
		caflag = 1;
        type1 = gen_op(op_a, *flag, index, index_type, i);
        i = gen_tmp_var(tmp, type1);
        emit(MOV, op_a, "", tmp);
        *flag = 0;
    }
    while(sym == PLUS || sym == MINU)
    {
        type1 = gen_op(op_a, *flag, index, index_type, i);
        op = (sym == PLUS) ? ADD : SUB;
        getsym();
        i = term(flag, &index, &index_type);
        type2 = gen_op(op_b, *flag, index, index_type, i);
        type1 = (type1 == 2 && type2 == 2) ? 2 : 1;
        i = gen_tmp_var(tmp, type1);
        emit(op, op_a, op_b, tmp);
        *flag = 0;
		caflag = 1;
    }
	if(rs)
    {
		if(caflag)
		{
			i = gen_tmp_var(op_b, type1);
			emit(NEG, tmp, "", op_b);
		}
		else
		{
			type1 = gen_op(op_a, *flag, index, index_type, i);
			//i = gen_tmp_var(tmp, type1);
			if(*flag > 1)
				i = -i;
			else
			{
				i = gen_tmp_var(op_b, type1);
				emit(NEG, op_a, "", op_b);
			}
			//*flag = 0;
		}
    }
    return i;
}

int term(int *flag, int *index, int *index_type)
{
    int i, type1, type2;
    enum quat_op op;
    char op_a[MAX_OP] = {0};
    char op_b[MAX_OP] = {0};
    char tmp[MAX_OP];
    i = factor(flag, index, index_type);
    while(sym == MULT || sym == DIV)
    {
        type1 = gen_op(op_a, *flag, *index, *index_type, i);
        op = (sym == MULT) ? MUL : DIVV;
        getsym();
        i = factor(flag, index, index_type);
        type2 = gen_op(op_b, *flag, *index, *index_type, i);
        type1 = (type1 == 2 && type2 == 2) ? 2 : 1;
        i = gen_tmp_var(tmp, type1);
        emit(op, op_a, op_b, tmp);
        *flag = 0;
    }
    return i;
}

int factor(int *flag, int *index, int *index_type)    //index表示数组的下标，flag=1表示返回的是否是数组，flag=2表示返回的是否为立即数，flag=3表示char立即数
{
    int i, j;
    char tmp[MAX_OP];
	char op[MAX_OP];
    *flag = 0;
    if(sym == IDEN)
    {
        i = position(iden);
        if(i < 0)
		{
            error(26);
			while(sym != RBRACE && sym != SEMICN && sym != IFTK && sym != DOTK && sym != FORTK)
				getsym();
			if(sym == SEMICN)
				getsym();
		}
        getsym();
        if(sym == LPARENT)  //有返回值的函数调用
        {
            if(table[i].kind != 3)
                error(26);
			else if(!table[i].type)
				error(33);
            getsym();
            i = invoke_func(i);
            i = gen_tmp_var(tmp, i);
            emit(MOV, "~eax", "", tmp);
        }
        else if(sym == LBRACK)  //数组
        {
            if(table[i].kind != 4)
                error(26);
            getsym();
            *index = expression(flag);
            *index_type = *flag;
            *flag = 1;
			gen_op(op, *flag, *index, *index_type, i);
			j = i;
			i = gen_tmp_var(tmp, table[i].type);
			emit(MOV, op, "", tmp);
			*flag = 0;
            if(*index_type && (table[j].int_value <= *index || *index < 0))	//其实很鸡肋，如果是表达式就没办法了
                error(27);
            if(sym == RBRACK)
                getsym();
            else
                error(12);
        }
        else
        {
            if(table[i].kind != 1 && table[i].kind != 2 && table[i].kind != 5)    //普通变量或常量
                error(26);
        }
    }
    else if(sym == INTCON || sym == CHARCON || sym == PLUS || sym == MINU)    //立即数常量
    {
        *flag = (sym == CHARCON) ? 3 : 2;
		int_num();
		i = (sym == INTCON) ? num : chcon;
        getsym();
    }
    else if(sym == LPARENT) //括号中的表达式
    {
        getsym();
        i = expression(flag);
        if(sym == RPARENT)
            getsym();
        else
            error(11);
    }
    return i;
}

void int_num()
{
	if(sym == PLUS || sym == MINU)
	{
		getsym();
		if(sym == INTCON)
		{
			if(num)
			{
				if_signed = 1;
				num = -num;
			}
			else
				error(9);
		}
		else
			error(32);
	}
	else if(sym == INTCON)
		if_signed = 0;
}

int gen_op(char *op, int flag, int index, int index_type, int i)    //生成四元式中规定格式的操作数
{
    int type;
    char tmp[MAX_OP], tmp2[MAX_OP];
    if(flag > 1)
    {
        sprintf(op, "%d", i);
        type = flag - 1;
    }
    else if(flag == 1)
    {
        if(i < lvx)
            strcpy(op, table[i].name);
        else
            sprintf(op, "%%%d", i - lvx);
        if(!index_type)
        {
            gen_op(tmp2, 0, 0, 0, index);
            sprintf(tmp, "[%s]", tmp2);
        }
        else
            sprintf(tmp, "[%d]", index);
        strcat(op, tmp);
        type = table[i].type;
    }
    else
    {
        if(table[i].kind == 6)
            strcpy(op, table[i].name);
        else if(i < lx)
            strcpy(op, table[i].name);
        else if(i >= lx && i < lvx)
            sprintf(op, "@%d", i - lx);
        else
            sprintf(op, "%%%d", i - lvx);
        type = table[i].type;
    }
    return type;
}
/*****end of sa.c*****/
////////////////////CODE////////////////////

////////////////////ASM_CODE////////////////////
/*****toasm.c*****/
void toasm()
{
    asm_head();
    asm_data();
    asm_code();
}

void insert_assign_table(char *name, int length, int type)
{
    strcpy(assign_table[addrx].name, name);
    assign_table[addrx].offset = esp;
    assign_table[addrx].tpye = type;
    esp += 4 * length;
    addrx++;
}

int search_addr(char *name)
{
    int i, k;
    int addr = 0;
    if(name[0] == '@')
    {
        k = atoi(&name[1]);      //...
        for(i = 0; i <= k; i++)
            addr += 4;
        addr += 4;
        return addr;
    }
    for(i = 0; i < addrx; i++)
		if(!strcmp(name, assign_table[i].name))
            return assign_table[i].offset;
    return -1;
}

int search_array_addr(const char *name, int *index_off, int *index_type)    //index_off是下标地址，index_type下标类型0参数1局部/临时变量
{
    int i, k, offset;
    char buff[MAX_NAME];
    for(i = 0, k = 0; name[i] != '\0' && name[i] != '['; i++, k++)
        buff[k] = name[i];
    if(name[i] == '\0') //不是数组
    {
        offset = -2;
        *index_off = -1;
        *index_type = -1;
        return offset;
    }
    buff[k] = '\0';
    offset = search_addr(buff);
    for(k = 0, i++; name[i] != ']'; i++, k++)
        buff[k] = name[i];
    buff[k] = '\0';
    *index_off = search_addr(buff);
    if(buff[0] == '@')
        *index_type = 0;
    else
        *index_type = 1;
    return offset;
}

int find_type(char *name)
{
    int i, k;
    char tmp[MAX_NAME];
    if(name[0] == '@')
        return 1;
    else if(name[0] == '$' || name[0] == '%')
    {
        for(i = 0, k = 0; name[i] != '[' && name[i] != '\0'; i++, k++)
            tmp[k] = name[i];
        tmp[k] = '\0';
        for(i = 0; i < addrx; i++)
			if(!strcmp(tmp, assign_table[i].name))
                return assign_table[i].tpye;
        return -1;
    }
    else
    {
        for(i = 0; i < tx; i++)
            if(!strcmp(name, table[i].name))
                return table[i].type;
        return -1;
    }
}

void asm_head()
{
    fprintf(asm_out, ".386\n");
    fprintf(asm_out, ".model flat, stdcall\n");
    fprintf(asm_out, "option casemap:none\n");
    fprintf(asm_out, "\n");
    fprintf(asm_out, "includelib msvcrt.lib\n");
    fprintf(asm_out, "printf  PROTO C:ptr sbyte,:vararg\n");
    fprintf(asm_out, "scanf  PROTO C:ptr sbyte,:vararg\n");
    fprintf(asm_out, "\n");
}

void asm_data()
{
	int i;
    fprintf(asm_out, ".data\n");
    while(quat_table[qq].op == CONST)
    {
        fprintf(asm_out, "_%s\tDD\t%s\n", quat_table[qq].operand_b, quat_table[qq].operand_r);
        qq++;
    }
    while(quat_table[qq].op == VAR)
    {
        if(!strcmp(quat_table[qq].operand_r, ""))
            fprintf(asm_out, "_%s\tDD\t?\n", quat_table[qq].operand_b);
        else
            fprintf(asm_out, "_%s\tDD\t%s dup(?)\n", quat_table[qq].operand_b, quat_table[qq].operand_r);
        qq++;
    }
    for(i = 0; i < strx; i++)
        fprintf(asm_out, "__str%d\tDB\t\"%s\",0\n", i, str_table[i]);
    fprintf(asm_out, "_CHAR\tDB\t\"%%c\",0\n");
    fprintf(asm_out, "_INT\tDB\t\"%%d\",0\n");
    fprintf(asm_out, "\n");
}

void asm_code()
{
    char tmp[MAX_NAME];
    int conx;
    int flag;
	int num;
	int i, j, k;
	char tmp_var[MAX_TMP_VAR][MAX_OP];
    fprintf(asm_out, ".code\n");
    while(quat_table[qq].op == FUNC || quat_table[qq].op == MAINF)
    {
        //fi = position(quat_table[qq].operand_a);
        if(quat_table[qq].op == MAINF)
        {
            flag = 0;
            fprintf(asm_out, "main\tproc\n");
        }
        else
        {
            flag = 1;
            fprintf(asm_out, "_%s\tproc\n", quat_table[qq].operand_a);
        }
        fprintf(asm_out, "\tpush\tebp\n");
        fprintf(asm_out, "\tmov\tebp,\tesp\n");
        //分配局部变量
        qq++;
        esp = 4;
        addrx = 0;
        addrtx = 4;
        while(quat_table[qq].op == CONST)
        {
            sprintf(tmp, "%%%d", addrx);
            if(!strcmp(quat_table[qq].operand_a, "INT"))
                insert_assign_table(tmp, 1, 1);
            else
                insert_assign_table(tmp, 1, 2);
            fprintf(asm_out, "\tsub\tesp,\t4\n");
            fprintf(asm_out, "\tmov\tdword ptr [esp],\t%s\n", quat_table[qq].operand_r);
            qq++;
        }
        conx = esp;
        while(quat_table[qq].op == VAR)
        {
            sprintf(tmp, "%%%d", addrx);
			if(strcmp(quat_table[qq].operand_r, ""))
				num = atoi(quat_table[qq].operand_r);
			else
				num = 1;
            if(!strcmp(quat_table[qq].operand_a, "INT"))
                insert_assign_table(tmp, num, 1);
            else
                insert_assign_table(tmp, num, 2);
            qq++;
        }
		fprintf(asm_out, "\tsub\tesp,\t%d\n", esp - conx);
        //保护现场
        if(flag)
        {
            fprintf(asm_out, "\tpush\tebx\n");
            fprintf(asm_out, "\tpush\tedi\n");
            fprintf(asm_out, "\tpush\tesi\n");
            esp += 12;
        }
        addrtx = esp;
		//分配临时变量空间
		k = 0;
		for(i = qq; quat_table[i].op != EFUNC && quat_table[i].op != EMAINF; i++)
		{
			if(quat_table[i].operand_r[0] != '$')
				continue;
			for(j = 0; j < k; j++)
				if(!strcmp(quat_table[i].operand_r, tmp_var[j]))
					break;
			if(j == k)
			{
				strcpy(tmp_var[k], quat_table[i].operand_r);
				k++;
			}
		}
		//esp += 4 * k;
		fprintf(asm_out, "\tsub\tesp,\t%d\n", 4*k);
        //代码执行
        while(quat_table[qq].op != EFUNC && quat_table[qq].op != EMAINF)
        {
            switch(quat_table[qq].op)
            {
            case ADD:
                asm_add();
                break;
            case SUB:
                asm_sub();
                break;
            case MUL:
                asm_mul();
                break;
            case DIVV:
                asm_divv();
                break;
            case NEG:
                asm_neg();
                break;
            case MOV:
                asm_mov();
                break;
            case JZ:
            case JNZ:
            case JL:
            case JLE:
            case JG:
            case JGE:
            case JE:
            case JNE:
                asm_jcdt(quat_table[qq].op);
                break;
            case RET:
                asm_ret();
                break;
            case WRITE:
                asm_write();
                break;
            case READ:
                asm_read();
                break;
            case PARAV:
                asm_parav();
                break;
            case CALL:
                asm_call();
                break;
            case JMP:
                asm_jump();
                break;
            case NOP:
                asm_nop();
                break;
            }
			qq++;
        }
        //恢复现场
		if(quat_table[qq].label != -1)
			fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
        if(flag)
        {
            fprintf(asm_out, "\tmov\tesp,\tebp\n");
            fprintf(asm_out, "\tsub\tesp,\t%d\n", addrtx - 4);
            fprintf(asm_out, "\tpop\tesi\n");
            fprintf(asm_out, "\tpop\tedi\n");
            fprintf(asm_out, "\tpop\tebx\n");
        }
        fprintf(asm_out, "\tmov\tesp,\tebp\n");
        fprintf(asm_out, "\tpop\tebp\n");
        fprintf(asm_out, "\tret\n");
        if(flag)
        {
            fprintf(asm_out, "_%s\tendp\n", quat_table[qq].operand_a);
            qq++;
        }
        else
        {
            fprintf(asm_out, "main\tendp\n");
            fprintf(asm_out, "end main\n");
            return ;
        }
    }
}

void asm_array(char *name, int offset, int index, int index_type)
{
    int i, k;
    char buff[MAX_NAME];
    fprintf(asm_out, "\tpush\teax\n");
	fprintf(asm_out, "\tpush\tecx\n");
    if(offset == -1 && index == -1)   //全局变量，下标为全局变量或立即数
    {
        for(i = 0, k = 0; name[i] != '['; i++, k++)
            buff[k] = name[i];
        buff[k] = '\0';
        fprintf(asm_out, "\tmov\tesi,\toffset _%s\n", buff);
        for(i++, k = 0; name[i] != ']'; i++, k++)
            buff[k] = name[i];
        buff[k] = '\0';
		if(!isalpha(buff[0]))
			fprintf(asm_out, "\tmov\teax,\t%s\n", buff);
		else
			fprintf(asm_out, "\tmov\teax,\t_%s\n", buff);
		fprintf(asm_out, "\tmov\tecx,\t4\n");
        fprintf(asm_out, "\timul\tecx\n");
        fprintf(asm_out, "\tadd\tesi,\teax\n");
    }
    else if(offset == -1 && index != -1)   //全局数组变量，下标为局部变量
    {
        for(i = 0, k = 0; name[i] != '['; i++, k++)
            buff[k] = name[i];
        buff[k] = '\0';
        fprintf(asm_out, "\tmov\tesi,\toffset _%s\n", buff);
        if(!index_type)
            fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", index);
        else
            fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", index);
        fprintf(asm_out, "\tmov\tecx,\t4\n");
        fprintf(asm_out, "\timul\tecx\n");
        fprintf(asm_out, "\tadd\tesi,\teax\n", index);
    }
    else if(offset != -1 && index == -1)    //局部数组变量，下标为立即数或全局变量
    {
        //参数不可能是数组
        fprintf(asm_out, "\tmov\tesi,\tebp\n");
        fprintf(asm_out, "\tsub\tesi,\t%d\n", offset);
        for(i = 0; name[i] != '['; i++)
            ;
        for(i++, k = 0; name[i] != ']'; i++, k++)
            buff[k] = name[i];
        buff[k] = '\0';
		if(!isalpha(buff[0]))
			fprintf(asm_out, "\tmov\teax,\t%s\n", buff);
		else
			fprintf(asm_out, "\tmov\teax,\t_%s\n", buff);
        fprintf(asm_out, "\tmov\tecx,\t4\n");
        fprintf(asm_out, "\timul\tecx\n");
        fprintf(asm_out, "\tsub\tesi,\teax\n");
    }
    else    //局部数组变量下标为局部变量
    {
        fprintf(asm_out, "\tmov\tesi,\tebp\n");
        if(!index_type)
            fprintf(asm_out, "\tadd\tesi,\t%d\n", index);
        else
            fprintf(asm_out, "\tsub\tesi,\t%d\n", index);
        fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        fprintf(asm_out, "\tmov\tesi,\tebp\n");
        fprintf(asm_out, "\tsub\tesi,\t%d\n", offset);
        fprintf(asm_out, "\tmov\tecx,\t4\n");
        fprintf(asm_out, "\timul\tecx\n");
        fprintf(asm_out, "\tsub\tesi,\teax\n");
    }
	fprintf(asm_out, "\tpop\tecx\n");
    fprintf(asm_out, "\tpop\teax\n");
}

void asm_add()
{
    int offset1, offset2, offset3;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(quat_table[qq].operand_r[0] == '$')
	{
        insert_assign_table(quat_table[qq].operand_r, 1, 1);
		//fprintf(asm_out, "\tsub\tesp,\t4\n");
	}
    offset1 = search_addr(quat_table[qq].operand_a);
    offset2 = search_addr(quat_table[qq].operand_b);
    offset3 = search_addr(quat_table[qq].operand_r);
    if(offset1 == -1)
    {
        //考虑数组情况
        offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset1 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset1, index, index_type);
            fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
    else
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
    if(offset2 == -1)
    {
        //考虑数组情况
        offset2 = search_array_addr(quat_table[qq].operand_b, &index, &index_type);
        if(offset2 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_b[0]))
				fprintf(asm_out, "\tadd\teax,\t%s\n", quat_table[qq].operand_b);
			else
				fprintf(asm_out, "\tadd\teax,\t_%s\n", quat_table[qq].operand_b);
		}
        else
        {
            asm_array(quat_table[qq].operand_b, offset2, index, index_type);
            fprintf(asm_out, "\tadd\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_b[0] == '@')
        fprintf(asm_out, "\tadd\teax,\tdword ptr [ebp+%d]\n", offset2);
    else
        fprintf(asm_out, "\tadd\teax,\tdword ptr [ebp-%d]\n", offset2);
    if(offset3 == -1)
    {
        //考虑数组情况
        offset3 = search_array_addr(quat_table[qq].operand_r, &index, &index_type);
        if(offset3 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_r[0]))
				fprintf(asm_out, "\tmov\t%s,\teax\n", quat_table[qq].operand_r);
			else
				fprintf(asm_out, "\tmov\t_%s,\teax\n", quat_table[qq].operand_r);
		}
        else
        {
            asm_array(quat_table[qq].operand_r, offset3, index, index_type);
            fprintf(asm_out, "\tmov\t[esi],\teax\n");
        }
    }
    else if(quat_table[qq].operand_r[0] == '@')
        fprintf(asm_out, "\tmov\tdword ptr [ebp+%d],\teax\n", offset3);
    else
        fprintf(asm_out, "\tmov\tdword ptr [ebp-%d],\teax\n", offset3);
}

void asm_sub()
{
    int offset1, offset2, offset3;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(quat_table[qq].operand_r[0] == '$')
	{
        insert_assign_table(quat_table[qq].operand_r, 1, 1);
		//fprintf(asm_out, "\tsub\tesp,\t4\n");
	}
    offset1 = search_addr(quat_table[qq].operand_a);
    offset2 = search_addr(quat_table[qq].operand_b);
    offset3 = search_addr(quat_table[qq].operand_r);
    if(offset1 == -1)
    {
        //考虑数组情况
        offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset1 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset1, index, index_type);
            fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
    else
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
    if(offset2 == -1)
    {
        //考虑数组情况
        offset2 = search_array_addr(quat_table[qq].operand_b, &index, &index_type);
        if(offset2 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_b[0]))
				fprintf(asm_out, "\tsub\teax,\t%s\n", quat_table[qq].operand_b);
			else
				fprintf(asm_out, "\tsub\teax,\t_%s\n", quat_table[qq].operand_b);
		}
        else
        {
            asm_array(quat_table[qq].operand_b, offset2, index, index_type);
            fprintf(asm_out, "\tsub\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_b[0] == '@')
        fprintf(asm_out, "\tsub\teax,\tdword ptr [ebp+%d]\n", offset2);
    else
        fprintf(asm_out, "\tsub\teax,\tdword ptr [ebp-%d]\n", offset2);
    if(offset3 == -1)
    {
        //考虑数组情况
        offset3 = search_array_addr(quat_table[qq].operand_r, &index, &index_type);
        if(offset3 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_r[0]))
				fprintf(asm_out, "\tmov\t%s,\teax\n", quat_table[qq].operand_r);
			else
				fprintf(asm_out, "\tmov\t_%s,\teax\n", quat_table[qq].operand_r);
		}
        else
        {
            asm_array(quat_table[qq].operand_r, offset3, index, index_type);
            fprintf(asm_out, "\tmov\t[esi],\teax\n");
        }
    }
    else if(quat_table[qq].operand_r[0] == '@')
        fprintf(asm_out, "\tmov\tdword ptr [ebp+%d],\teax\n", offset3);
    else
        fprintf(asm_out, "\tmov\tdword ptr [ebp-%d],\teax\n", offset3);
}

void asm_mul()
{
    int offset1, offset2, offset3;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(quat_table[qq].operand_r[0] == '$')
	{
        insert_assign_table(quat_table[qq].operand_r, 1, 1);
		//fprintf(asm_out, "\tsub\tesp,\t4\n");
	}
    offset1 = search_addr(quat_table[qq].operand_a);
    offset2 = search_addr(quat_table[qq].operand_b);
    offset3 = search_addr(quat_table[qq].operand_r);
    if(offset1 == -1)
    {
        //考虑数组情况
        offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset1 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset1, index, index_type);
            fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
    else
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
	fprintf(asm_out, "\tmov\tedx,\teax\n");
	fprintf(asm_out, "\tsar\tedx,\t31\n");
    if(offset2 == -1)
    {
        //考虑数组情况
        offset2 = search_array_addr(quat_table[qq].operand_b, &index, &index_type);
        if(offset2 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_b[0]))
			{
				fprintf(asm_out, "\tmov\tecx,\t%s\n", quat_table[qq].operand_b);
				fprintf(asm_out, "\timul\tecx\n");
			}
			else
				fprintf(asm_out, "\timul\t_%s\n", quat_table[qq].operand_b);
		}
        else
        {
            asm_array(quat_table[qq].operand_b, offset2, index, index_type);
            fprintf(asm_out, "\timul\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_b[0] == '@')
        fprintf(asm_out, "\timul\tdword ptr [ebp+%d]\n", offset2);
    else
        fprintf(asm_out, "\timul\tdword ptr [ebp-%d]\n", offset2);
    if(offset3 == -1)
    {
        //考虑数组情况
        offset3 = search_array_addr(quat_table[qq].operand_r, &index, &index_type);
        if(offset3 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_r[0]))
				fprintf(asm_out, "\tmov\t%s,\teax\n", quat_table[qq].operand_r);
			else
				fprintf(asm_out, "\tmov\t_%s,\teax\n", quat_table[qq].operand_r);
		}
        else
        {
            asm_array(quat_table[qq].operand_r, offset3, index, index_type);
            fprintf(asm_out, "\tmov\t[esi],\teax\n");
        }
    }
    else if(quat_table[qq].operand_r[0] == '@')
        fprintf(asm_out, "\tmov\tdword ptr [ebp+%d],\teax\n", offset3);
    else
        fprintf(asm_out, "\tmov\tdword ptr [ebp-%d],\teax\n", offset3);
}

void asm_divv()
{
    int offset1, offset2, offset3;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(quat_table[qq].operand_r[0] == '$')
	{
        insert_assign_table(quat_table[qq].operand_r, 1, 1);
		//fprintf(asm_out, "\tsub\tesp,\t4\n");
	}
    offset1 = search_addr(quat_table[qq].operand_a);
    offset2 = search_addr(quat_table[qq].operand_b);
    offset3 = search_addr(quat_table[qq].operand_r);
    if(offset1 == -1)
    {
        //考虑数组情况
        offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset1 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset1, index, index_type);
            fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
    else
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
	fprintf(asm_out, "\tmov\tedx,\teax\n");
	fprintf(asm_out, "\tsar\tedx,\t31\n");
    if(offset2 == -1)
    {
        //考虑数组情况
        offset2 = search_array_addr(quat_table[qq].operand_b, &index, &index_type);
        if(offset2 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_b[0]))
			{
				fprintf(asm_out, "\tmov\tecx,\t%s\n", quat_table[qq].operand_b);
				fprintf(asm_out, "\tidiv\tecx\n");
			}
			else
				fprintf(asm_out, "\tidiv\t_%s\n", quat_table[qq].operand_b);
		}
        else
        {
            asm_array(quat_table[qq].operand_b, offset2, index, index_type);
            fprintf(asm_out, "\tidiv\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_b[0] == '@')
        fprintf(asm_out, "\tidiv\tdword ptr [ebp+%d]\n", offset2);
    else
        fprintf(asm_out, "\tidiv\tdword ptr [ebp-%d]\n", offset2);
    if(offset3 == -1)
    {
        //考虑数组情况
        offset3 = search_array_addr(quat_table[qq].operand_r, &index, &index_type);
        if(offset3 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_r[0]))
				fprintf(asm_out, "\tmov\t%s,\teax\n", quat_table[qq].operand_r);
			else
				fprintf(asm_out, "\tmov\t_%s,\teax\n", quat_table[qq].operand_r);
		}
        else
        {
            asm_array(quat_table[qq].operand_r, offset3, index, index_type);
            fprintf(asm_out, "\tmov\t[esi],\teax\n");
        }
    }
    else if(quat_table[qq].operand_r[0] == '@')
        fprintf(asm_out, "\tmov\tdword ptr [ebp+%d],\teax\n", offset3);
    else
        fprintf(asm_out, "\tmov\tdword ptr [ebp-%d],\teax\n", offset3);
}

void asm_neg()
{
    int offset;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    offset = search_addr(quat_table[qq].operand_a);
    if(offset == -1)
    {
        //考虑数组情况
        offset = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tneg\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tneg\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset, index, index_type);
            fprintf(asm_out, "\tneg\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tneg\tdword ptr [ebp+%d]\n", offset);
    else
        fprintf(asm_out, "\tneg\tdword ptr [ebp-%d]\n", offset);
}

void asm_mov()
{
    int offset1, offset2;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(quat_table[qq].operand_r[0] == '$')
	{
        insert_assign_table(quat_table[qq].operand_r, 1, 1);
		//fprintf(asm_out, "\tsub\tesp,\t4\n");
	}
    offset1 = search_addr(quat_table[qq].operand_a);
    offset2 = search_addr(quat_table[qq].operand_r);
    if(quat_table[qq].operand_a[0] == '~')  //函数返回值是源操作数的特殊指令
        ;
    else if(offset1 == -1)
    {
        //考虑数组情况
        offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset1 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset1, index, index_type);
            fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
    else
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
    if(offset2 == -1)
    {
        //考虑数组情况
        offset2 = search_array_addr(quat_table[qq].operand_r, &index, &index_type);
        if(offset2 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_r[0]))
				fprintf(asm_out, "\tmov\t%s,\teax\n", quat_table[qq].operand_r);
			else
				fprintf(asm_out, "\tmov\t_%s,\teax\n", quat_table[qq].operand_r);
		}
        else
        {
            asm_array(quat_table[qq].operand_r, offset2, index, index_type);
            fprintf(asm_out, "\tmov\t[esi],\teax\n");
        }
    }
    else if(quat_table[qq].operand_r[0] == '@')
        fprintf(asm_out, "\tmov\tdword ptr [ebp+%d],\teax\n", offset2);
    else
        fprintf(asm_out, "\tmov\tdword ptr [ebp-%d],\teax\n", offset2);
}

void asm_jcdt(enum quat_op op)
{
    int offset1, offset2;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    offset1 = search_addr(quat_table[qq].operand_a);
    if(offset1 == -1)
    {
        //考虑数组情况
        offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
        if(offset1 == -2)   //非数组，是立即数或全局变量
		{
			if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
		}
        else
        {
            asm_array(quat_table[qq].operand_a, offset1, index, index_type);
            fprintf(asm_out, "\tmov\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
    else
        fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);

    if(strcmp(quat_table[qq].operand_b, ""))
    {
        offset2 = search_addr(quat_table[qq].operand_b);
        if(offset2 == -1)
        {
            //考虑数组情况
            offset2 = search_array_addr(quat_table[qq].operand_b, &index, &index_type);
            if(offset2 == -2)   //非数组，是立即数或全局变量
			{
				if(!isalpha(quat_table[qq].operand_b[0]))
					fprintf(asm_out, "\tmov\tecx,\t%s\n", quat_table[qq].operand_b);
				else
					fprintf(asm_out, "\tmov\tecx,\t_%s\n", quat_table[qq].operand_b);
			}
            else
            {
                asm_array(quat_table[qq].operand_b, offset2, index, index_type);
                fprintf(asm_out, "\tmov\tecx,\t[esi]\n");
            }
        }
        else if(quat_table[qq].operand_b[0] == '@')
            fprintf(asm_out, "\tmov\tecx,\tdword ptr [ebp+%d]\n", offset2);
        else
            fprintf(asm_out, "\tmov\tecx,\tdword ptr [ebp-%d]\n", offset2);
    }

    switch(op)
    {
    case JZ:
        fprintf(asm_out, "\tcmp\teax,\t0\n");
        fprintf(asm_out, "\tjz\t%s\n", quat_table[qq].operand_r);
        break;
    case JNZ:
        fprintf(asm_out, "\tcmp\teax,\t0\n");
        fprintf(asm_out, "\tjnz\t%s\n", quat_table[qq].operand_r);
        break;
    case JL:
        fprintf(asm_out, "\tcmp\teax,\tecx\n");
        fprintf(asm_out, "\tjl\t%s\n", quat_table[qq].operand_r);
        break;
    case JLE:
        fprintf(asm_out, "\tcmp\teax,\tecx\n");
        fprintf(asm_out, "\tjle\t%s\n", quat_table[qq].operand_r);
        break;
    case JG:
        fprintf(asm_out, "\tcmp\teax,\tecx\n");
        fprintf(asm_out, "\tjg\t%s\n", quat_table[qq].operand_r);
        break;
    case JGE:
        fprintf(asm_out, "\tcmp\teax,\tecx\n");
        fprintf(asm_out, "\tjge\t%s\n", quat_table[qq].operand_r);
        break;
    case JE:
        fprintf(asm_out, "\tcmp\teax,\tecx\n");
        fprintf(asm_out, "\tje\t%s\n", quat_table[qq].operand_r);
        break;
    case JNE:
        fprintf(asm_out, "\tcmp\teax,\tecx\n");
        fprintf(asm_out, "\tjne\t%s\n", quat_table[qq].operand_r);
        break;
    }
}

void asm_ret()
{
    int offset1;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(strcmp(quat_table[qq].operand_a, ""))
    {
        offset1 = search_addr(quat_table[qq].operand_a);
        if(offset1 == -1)
        {
            //考虑数组情况
            offset1 = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
            if(offset1 == -2)   //非数组，是立即数或全局变量
			{
				if(!isalpha(quat_table[qq].operand_a[0]))
				fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_a);
			else
				fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_a);
			}
            else
            {
                asm_array(quat_table[qq].operand_a, offset1, index, index_type);
                fprintf(asm_out, "\tmov\teax,\t[esi]\n");
            }
        }
        else if(quat_table[qq].operand_a[0] == '@')
            fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
        else
            fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
    }
    fprintf(asm_out, "\tmov\tesp,\tebp\n");
    fprintf(asm_out, "\tsub\tesp,\t%d\n", addrtx);
    fprintf(asm_out, "\tpop\tesi\n");
    fprintf(asm_out, "\tpop\tedi\n");
    fprintf(asm_out, "\tpop\tebx\n");
    fprintf(asm_out, "\tmov\tesp,\tebp\n");
    fprintf(asm_out, "\tpop\tebp\n");
    fprintf(asm_out, "\tret\n");
}

void asm_write()
{
    int offset1;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    if(strcmp(quat_table[qq].operand_a, ""))
    {
        fprintf(asm_out, "\tlea\teax,\t%s\n", quat_table[qq].operand_a);
        fprintf(asm_out, "\tpush\teax\n");
        fprintf(asm_out, "\tcall\tprintf\n");
        fprintf(asm_out, "\tadd\tesp,\t4\n");
    }
    if(strcmp(quat_table[qq].operand_b, ""))
    {
        offset1 = search_addr(quat_table[qq].operand_b);
        if(offset1 == -1)
        {
            //考虑数组情况
            offset1 = search_array_addr(quat_table[qq].operand_b, &index, &index_type);
            if(offset1 == -2)   //非数组，是立即数或全局变量
			{
				if(!isalpha(quat_table[qq].operand_b[0]))
					fprintf(asm_out, "\tmov\teax,\t%s\n", quat_table[qq].operand_b);
				else
					fprintf(asm_out, "\tmov\teax,\t_%s\n", quat_table[qq].operand_b);
			}
            else
            {
                asm_array(quat_table[qq].operand_b, offset1, index, index_type);
                fprintf(asm_out, "\tmov\teax,\t[esi]\n");
            }
        }
        else if(quat_table[qq].operand_b[0] == '@')
            fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp+%d]\n", offset1);
        else
            fprintf(asm_out, "\tmov\teax,\tdword ptr [ebp-%d]\n", offset1);
        fprintf(asm_out, "\tpush\teax\n");
        if(find_type(quat_table[qq].operand_b) == 2)
            fprintf(asm_out, "\tlea\teax,\t_CHAR\n");
        else
            fprintf(asm_out, "\tlea\teax,\t_INT\n");
        fprintf(asm_out, "\tpush\teax\n");
        fprintf(asm_out, "\tcall\tprintf\n");
        fprintf(asm_out, "\tadd\tesp,\t8\n");
    }
}

void asm_read()
{
    int offset;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    offset = search_addr(quat_table[qq].operand_r);
    if(offset == -1)
    {
        //考虑数组情况
        offset = search_array_addr(quat_table[qq].operand_r, &index, &index_type);
        if(offset == -2)   //非数组，是全局变量
            fprintf(asm_out, "\tlea\teax,\t_%s\n", quat_table[qq].operand_r);
        else
        {
            asm_array(quat_table[qq].operand_r, offset, index, index_type);
            fprintf(asm_out, "\tlea\teax,\t[esi]\n");
        }
    }
    else if(quat_table[qq].operand_a[0] == '@')
        fprintf(asm_out, "\tlea\teax,\tdword ptr [ebp+%d]\n", offset);
    else
        fprintf(asm_out, "\tlea\teax,\tdword ptr [ebp-%d]\n", offset);
    fprintf(asm_out, "\tpush\teax\n");
    if(find_type(quat_table[qq].operand_r) == 2)
        fprintf(asm_out, "\tlea\teax,\t_CHAR\n");
    else
        fprintf(asm_out, "\tlea\teax,\t_INT\n");
    fprintf(asm_out, "\tpush\teax\n");
    fprintf(asm_out, "\tcall\tscanf\n");
    fprintf(asm_out, "\tadd\tesp,\t8\n");
}

void asm_parav()
{
    int offset;
    int index, index_type;
    if(quat_table[qq].label != -1)
        fprintf(asm_out, "%s:\n", label_table[quat_table[qq].label]);
    do
    {
        offset = search_addr(quat_table[qq].operand_a);
        if(offset == -1)
        {
            //考虑数组情况
            offset = search_array_addr(quat_table[qq].operand_a, &index, &index_type);
            if(offset == -2)   //非数组，是立即数或全局变量
			{
				if(!isalpha(quat_table[qq].operand_a[0]))
					fprintf(asm_out, "\tpush\t%s\n", quat_table[qq].operand_a);
				else
					fprintf(asm_out, "\tpush\t_%s\n", quat_table[qq].operand_a);
			}
            else
            {
                asm_array(quat_table[qq].operand_a, offset, index, index_type);
                fprintf(asm_out, "\tpush\t[esi]\n");
            }
        }
        else if(quat_table[qq].operand_a[0] == '@')
            fprintf(asm_out, "\tpush\tdword ptr [ebp+%d]\n", offset);
        else
            fprintf(asm_out, "\tpush\tdword ptr [ebp-%d]\n", offset);
        qq++;
    } while(quat_table[qq].op == PARAV);
	qq--;
}

void asm_call()
{
    int i = 0, k = 0;
    fprintf(asm_out, "\tcall\t_%s\n", quat_table[qq].operand_a);
    i = position(quat_table[qq].operand_a);
    for(i++; table[i].kind == 5; i++)
        k++;
    fprintf(asm_out, "\tadd\tesp,\t%d\n", k*4);
}

void asm_jump()
{
    fprintf(asm_out, "\tjmp\t%s\n", quat_table[qq].operand_r);
}

void asm_nop()
{
    fprintf(asm_out, "\tnop\n");
}
/*****end of toasm.c*****/
////////////////////ASM_CODE////////////////////

////////////////////OPT_CODE////////////////////
/*****block.c*****/
int proc_block(int i)
{
    int j, k = 0, q, r;    //k为blocks表内层指针
    while(quat_table[i].op == CONST || quat_table[i].op == VAR)
        i++;
    blocks[blx][k++] = i;
    while(quat_table[i].op != EFUNC && quat_table[i].op != EMAINF)
    {
        if(quat_table[i].op == JZ || quat_table[i].op == JNZ ||
           quat_table[i].op == JL || quat_table[i].op == JLE ||
           quat_table[i].op == JG || quat_table[i].op == JGE ||
           quat_table[i].op == JE || quat_table[i].op == JNE ||
           quat_table[i].op == JMP || quat_table[i].op == RET)
        {
            if(quat_table[i+1].op != EFUNC && quat_table[i+1].op != EMAINF) //RET/跳转语句后一句
            {
                for(q = 0; q < k; q++)  //是否已经存在
                    if(blocks[blx][q] == i + 1)
                        break;
                if(q == k)
                    blocks[blx][k++] = i + 1;
            }
            if(quat_table[i].op != RET)
            {
                for(j = 0; j < qtx; j++)    //寻找跳转到的语句
                {
                    if(!strcmp(quat_table[i].operand_r, label_table[quat_table[j].label]))
                    {
                        for(q = 0; q < k; q++)  //是否已经存在
                            if(blocks[blx][q] == j)
                                break;
                        if(q == k)
                            blocks[blx][k++] = j;
                        break;
                    }
                }
            }
        }
        i++;
    }
    for(j = 0; j < k; j++)  //冒泡排序
        for(q = k - 1; q > j; q--)
            if(blocks[blx][q] < blocks[blx][q-1])
            {
                r = blocks[blx][q];
                blocks[blx][q] = blocks[blx][q-1];
                blocks[blx][q-1] = r;
            }
    blocks[blx][k++] = i;   //记录函数结束点EFUNC
    blocks[blx][k] = -1;
    blx++;
    return i;
}

void divide_block()
{
    int i;
    for(i = 0; i < qtx; i++)
        if(quat_table[i].op == FUNC || quat_table[i].op == MAINF)
            break;
    while(i < qtx && (quat_table[i].op == FUNC || quat_table[i].op == MAINF))
    {
        i++;
        i = proc_block(i);
        i++;
    }
}

void copy_oquat()
{
    int i, j;
    for(i = 0, j = 0; j < oqtx; j++)
        quat_table[i++] = opt_quat[j];
    qtx = i;
}
/*****end of block.c*****/

/*****dag.c*****/
void opt_dag()
{
    int i, j, r = 0;    //r为原四元式遍历量
    oqtx = 0;
    dtx = 0;
    ntx = 0;
    for(i = 0; i < MAX_TABLE; i++)
        opt_quat[i].label = -1;
    for(i = 0; i < blx; i++)
    {
        while(r < blocks[i][0])
        {
            opt_quat[oqtx++] = quat_table[r];
            r++;
        }
        for(j = 0; blocks[i][j+1] != -1; j++)
            build_dag(blocks[i][j], blocks[i][j+1]);
        r = blocks[i][j];
    }
    while(r < qtx)
	{
        opt_quat[oqtx++] = quat_table[r];
		r++;
	}
    copy_oquat();
}

void write_dag()    //自带清零
{
    int STACK[MAX_STACK];
    int mid_nodes[MAX_TABLE];
	char op[MAX_OP];
    int stk = 0, mid_num, i, j, last, bstk;
    mid_num = select_mid_nodes(mid_nodes);
    last = mid_num - 1;
    while(last >= 0)
    {
        bstk = stk;
        STACK[stk++] = mid_nodes[last];
        i = mid_nodes[last];
        while(dag_table[i].lchild != -1 && !has_parent(dag_table[i].lchild, last, mid_nodes))
        {
            STACK[stk++] = dag_table[i].lchild;
            i = dag_table[i].lchild;
        }
        for(i = bstk; i < stk; i++)
			for(j = last; j >= 0; j--)
				if(mid_nodes[j] == STACK[i])
				{
					mid_nodes[j] = -1;
					break;
				}
        while(last >= 0 && mid_nodes[last] == -1)
            last--;
    }
    for(i = stk - 1; i >= 0; i--)
        write_opt_quat(STACK[i]);
	//处理叶子节点
	for(i = 0; i < dtx; i++)
		if(dag_table[i].lchild == -1)
		{
			for(j = 0; j < ntx; j++)
				if(node_table[j].num == i && node_table[j].name[0] != '$')
				{
					if(dag_table[i].op[0] == '_')
						strcpy(op, &dag_table[i].op[1]);
					else
						strcpy(op, dag_table[i].op);
					if(strcmp(op, node_table[j].name))
						insert_opt_quat_table(MOV, op, "", node_table[j].name);
				}
		}
    dtx = 0;
    ntx = 0;
}

void write_opt_quat(int node)
{
    int lchild, rchild, i, k, flag;
    char op_a[MAX_OP], op_b[MAX_OP], tmp[MAX_OP];
    enum quat_op op;
    lchild = dag_table[node].lchild;
    rchild = dag_table[node].rchild;
    if(dag_table[lchild].lchild == -1)  //左孩子是叶结点
    {
        if(dag_table[lchild].op[0] == '_')
            strcpy(op_a, &dag_table[lchild].op[1]);
        else
            strcpy(op_a, dag_table[lchild].op);
    }
    else    //左孩子是中间结点
    {
        for(i = 0, k = -1; i < ntx; i++)
        {
            if(node_table[i].num == lchild)
            {
                k = i;
                if(node_table[i].name[0] != '$')
                    break;
            }
        }
        strcpy(op_a, node_table[k].name);
    }

    if(rchild != -1)    //存在右孩子
    {
        if(dag_table[rchild].lchild == -1)  //右孩子是叶结点
        {
            if(dag_table[rchild].op[0] == '_')
                strcpy(op_b, &dag_table[rchild].op[1]);
            else
                strcpy(op_b, dag_table[rchild].op);
        }
        else    //右孩子是中间结点
        {
            for(i = 0, k = -1; i < ntx; i++)
            {
                if(node_table[i].num == rchild)
                {
                    k = i;
                    if(node_table[i].name[0] != '$')
                        break;
                }
            }
            strcpy(op_b, node_table[k].name);
        }
    }
    else
        op_b[0] = '\0';

    if(!strcmp(dag_table[node].op, "ADD"))
        op = ADD;
    else if(!strcmp(dag_table[node].op, "SUB"))
        op = SUB;
    else if(!strcmp(dag_table[node].op, "MUL"))
        op = MUL;
    else if(!strcmp(dag_table[node].op, "DIVV"))
        op = DIVV;
    else if(!strcmp(dag_table[node].op, "NEG"))
        op = NEG;
    else if(!strcmp(dag_table[node].op, "[]") || !strcmp(dag_table[node].op, "[]="))
        op = MOV;
	flag = 0;
    for(i = 0, k = -1; i < ntx; i++)
    {
        if(node_table[i].num == node)
        {
			if(node_table[i].name[0] == '~')
				insert_opt_quat_table(NOP, "", "", "");
            else if(node_table[i].name[0] == '$')
                k = i;
            else
            {
				flag = 1;
                if(op == MOV)
                {
                    if(!strcmp(dag_table[node].op, "[]"))
                    {
                        sprintf(tmp, "%s[%s]", op_a, op_b);
                        insert_opt_quat_table(op, tmp, "", node_table[i].name);
                    }
                    else
                    {
                        sprintf(tmp, "%s[%s]", node_table[i].name, op_a);
                        insert_opt_quat_table(op, op_b, "", tmp);
                    }
                }
                else
                    insert_opt_quat_table(op, op_a, op_b, node_table[i].name);
            }
        }
    }
    if(!flag)
    {
        if(op == MOV)
        {
            if(!strcmp(dag_table[node].op, "[]"))
            {
                sprintf(tmp, "%s[%s]", op_a, op_b);
                insert_opt_quat_table(op, tmp, "", node_table[k].name);
            }
            else
            {
                sprintf(tmp, "%s[%s]", node_table[k].name, op_a);
                insert_opt_quat_table(op, op_b, "", tmp);
            }
        }
        else
            insert_opt_quat_table(op, op_a, op_b, node_table[k].name);
    }
}

void insert_opt_quat_table(enum quat_op op, char *op_a, char *op_b, char *op_r)
{
    opt_quat[oqtx].op = op;
    strcpy(opt_quat[oqtx].operand_a, op_a);
    strcpy(opt_quat[oqtx].operand_b, op_b);
    strcpy(opt_quat[oqtx].operand_r, op_r);
    oqtx++;
}

int has_parent(int node, int last, int mid_nodes[])
{
    int i;
    for(i = last; i >= 0; i--)
        if(dag_table[mid_nodes[i]].lchild == node || 
			dag_table[mid_nodes[i]].rchild == node)
            return 1;
    return 0;
}

int select_mid_nodes(int mid_nodes[])
{
    int i, k = 0;
    for(i = 0; i < dtx; i++)
        if(dag_table[i].lchild != -1)
            mid_nodes[k++] = i;
    return k;
}

void build_dag(const int s, const int e)
{
    int i;
    int child1, child2, parent, parin;
    char op[MAX_OP], index[MAX_OP], name[MAX_OP];
    if(quat_table[s].label != -1)
        opt_quat[oqtx].label = quat_table[s].label;
    for(i = s; i < e; i++)
    {
        if(quat_table[i].op == ADD || quat_table[i].op == SUB
           || quat_table[i].op == MUL || quat_table[i].op == DIVV)
        {
            child1 = search_node_table(quat_table[i].operand_a);
            if(child1 == -1)
            {
                gen_node_op(quat_table[i].operand_a, CONST, name);
                child1 = insert_dag_table(name, -1, -1);
				if(name[0] == '_')
					insert_node_table(quat_table[i].operand_a, child1);
            }
            else
                child1 = node_table[child1].num;
            child2 = search_node_table(quat_table[i].operand_b);
            if(child2 == -1)
            {
                gen_node_op(quat_table[i].operand_b, CONST, name);
                child2 = insert_dag_table(name, -1, -1);
				if(name[0] == '_')
					insert_node_table(quat_table[i].operand_b, child2);
            }
            else
                child2 = node_table[child2].num;
            gen_node_op("", quat_table[i].op, name);
            parent = search_dag_table(name, child1, child2);
            if(parent == -1)
                parent = insert_dag_table(name, child1, child2);
            parin = search_node_table(quat_table[i].operand_r);
            if(parin == -1)
                insert_node_table(quat_table[i].operand_r, parent);
            else
                node_table[parin].num = parent;
        }
        else if(quat_table[i].op == NEG)
        {
            child1 = search_node_table(quat_table[i].operand_a);
            if(child1 == -1)
            {
                gen_node_op(quat_table[i].operand_a, CONST, name);
                child1 = insert_dag_table(name, -1, -1);
				if(name[0] == '_')
					insert_node_table(quat_table[i].operand_a, child1);
            }
            else
                child1 = node_table[child1].num;
            gen_node_op("", quat_table[i].op, name);
            parent = search_dag_table(name, child1, -1);
            if(parent == -1)
                parent = insert_dag_table(name, child1, -1);
            parin = search_node_table(quat_table[i].operand_r);
            if(parin == -1)
                insert_node_table(quat_table[i].operand_r, parent);
            else
                node_table[parin].num = parent;
        }
        else if(quat_table[i].op == MOV)
        {
			if(!strcmp(quat_table[i].operand_a, "~eax"))
				opt_quat[oqtx++] = quat_table[i];
            else if(test_array(quat_table[i].operand_a, op, index))
            {
                child1 = search_node_table(op);
                if(child1 == -1)
                {
                    gen_node_op(op, CONST, name);
                    child1 = insert_dag_table(name, -1, -1);
					if(name[0] == '_')
						insert_node_table(op, child1);
                }
                else
                    child1 = node_table[child1].num;
                child2 = search_node_table(index);
                if(child2 == -1)
                {
                    gen_node_op(index, CONST, name);
                    child2 = insert_dag_table(name, -1, -1);
					if(name[0] == '_')
						insert_node_table(index, child2);
                }
                else
                    child2 = node_table[child2].num;
                gen_node_op("[]", VAR, name);
                parent = search_dag_table(name, child1, child2);
                if(parent == -1)
                    parent = insert_dag_table(name, child1, child2);
                insert_node_table(quat_table[i].operand_r, parent);
            }
            else if(test_array(quat_table[i].operand_r, op, index))
            {
                child1 = search_node_table(index);
                if(child1 == -1)
                {
                    gen_node_op(index, CONST, name);
                    child1 = insert_dag_table(name, -1, -1);
					if(name[0] == '_')
						insert_node_table(index, child1);
                }
                else
                    child1 = node_table[child1].num;
                child2 = search_node_table(quat_table[i].operand_a);
                if(child2 == -1)
                {
                    gen_node_op(quat_table[i].operand_a, CONST, name);
                    child2 = insert_dag_table(name, -1, -1);
					if(name[0] == '_')
						insert_node_table(quat_table[i].operand_a, child2);
                }
                else
                    child2 = node_table[child2].num;
                gen_node_op("[]=", VAR, name);
                parent = search_dag_table(name, child1, child2);
                if(parent == -1)
                    parent = insert_dag_table(name, child1, child2);
                insert_node_table(op, parent);
            }
            else
            {
                child1 = search_node_table(quat_table[i].operand_a);
                if(child1 == -1)
                {
                    gen_node_op(quat_table[i].operand_a, CONST, name);
                    child1 = insert_dag_table(name, -1, -1);
					if(name[0] == '_')
						insert_node_table(quat_table[i].operand_a, child1);
                }
                else
                    child1 = node_table[child1].num;
                parin = search_node_table(quat_table[i].operand_r);
                if(parin == -1)
                    insert_node_table(quat_table[i].operand_r, child1);
                else
                    node_table[parin].num = child1;
            }
        }
        else if(quat_table[i].op == READ || quat_table[i].op == WRITE)
        {
            write_dag();
            opt_quat[oqtx++] = quat_table[i];
        }
        else if(quat_table[i].op == NOP)    //按赋值方式处理
        {
            insert_node_table("~", dtx - 1);
        }
        else if(quat_table[i].op == PARAV)
        {
            write_dag();
            do
            {
                opt_quat[oqtx++] = quat_table[i];
                i++;
            } while(quat_table[i].op != CALL);
            opt_quat[oqtx++] = quat_table[i];
        }
        else    //JMP类指令
        {
            write_dag();
            opt_quat[oqtx++] = quat_table[i];
        }
    }
    if(dtx || ntx)
        write_dag();
}

int test_array(const char *name, char *op, char *index)
{
    int i, k;
    for(i = 0, k = 0; name[i] != '\0' && name[i] != '['; i++)
        op[k++] = name[i];
    op[k] = '\0';
    if(!name[i])
        return 0;
    for(i++, k = 0; name[i] != ']'; i++)
        index[k++] = name[i];
    index[k] = '\0';
    return 1;
}

//产生节点名字
void gen_node_op(const char *name, enum quat_op op, char *node_name)  //op为CONST表示叶结点
{
    if(op == CONST)   //叶结点
    {
        if(isdigit(name[0]))
            strcpy(node_name, name);
        else
            sprintf(node_name, "_%s", name);
    }
    else if(op == VAR)  //数组运算
        strcpy(node_name, name);
    else    //中间结点
        strcpy(node_name, quat_op_name[op]);
}

int search_node_table(char *name)
{
    int i;
    for(i = 0; i < ntx; i++)
        if(!strcmp(name, node_table[i].name))
            return i;
    return -1;
}

int search_dag_table(char *op, int lchild, int rchild)
{
    int i;
    for(i = 0; i < dtx; i++)
        if(!strcmp(op, dag_table[i].op) &&
           dag_table[i].lchild == lchild && dag_table[i].rchild == rchild)
           return i;
    return -1;
}

int insert_dag_table(char *op, int lchild, int rchild)
{
    strcpy(dag_table[dtx].op, op);
    dag_table[dtx].lchild = lchild;
    dag_table[dtx].rchild = rchild;
    dtx++;
    return dtx - 1;
}

void insert_node_table(char *name, int num)
{
    strcpy(node_table[ntx].name, name);
    node_table[ntx].num = num;
    ntx++;
}
/*****end of dag.c*****/

/*****local.c*****/
void opt_local()
{
    FILE *asm_in;
    char line1[MAX_LINE], line2[MAX_LINE];
    char op1[MAX_OP], op2[MAX_OP];
    char a1[MAX_NAME], b1[MAX_NAME], a2[MAX_NAME], b2[MAX_NAME];
    int i;
    fclose(asm_out);
    system("rename a.asm ~a.asm");
    asm_in = fopen("~a.asm", "r");
    asm_out = fopen("a.asm", "w");
    while(fgets(line1, MAX_LINE - 1, asm_in) && strncmp(line1, ".code", 5))
        fputs(line1, asm_out);
	fputs(line1, asm_out);
    fgets(line1, MAX_LINE - 1, asm_in);
    while(fgets(line2, MAX_LINE - 1, asm_in))
    {
		sscanf(line1, "%s", op1);
        if(!strcmp(op1, "mov"))
        {
            sscanf(line2, "%s", op2);
            if(!strcmp(op2, "mov"))
            {
				i = 0;
				parse_line(line1, a1, b1);
				parse_line(line2, a2, b2);
                if(!strcmp(a1, b2) && !strcmp(a2, b1))
                    fgets(line2, MAX_LINE - 1, asm_in);
            }
        }
        fputs(line1, asm_out);
        strcpy(line1, line2);
    }
	fputs(line1, asm_out);
    fclose(asm_in);
    system("del ~a.asm");
}

void parse_line(const char *line, char *a, char *b)
{
	int i = 0, k;
	while(isspace(line[i]))
		i++;
	i += 3;
	while(isspace(line[i]))
		i++;
	k = 0;
	while(line[i] != ',')
		a[k++] = line[i++];
	a[k] = '\0';
	i++;
	while(isspace(line[i]))
		i++;
	k = 0;
	while(line[i] != '\n')
		b[k++] = line[i++];
	b[k] = '\0';
}
/*****end of local.c*****/
////////////////////OPT_CODE////////////////////

////////////////////MAIN////////////////////
int main()
{
    int i = 0;
	char source[MAX_FILE_NAME];
	printf("请输入源文件名：");
	gets(source);
    in = fopen(source, "r");
    out = fopen("quat.txt", "w");
    err = fopen("err.txt", "w");
	asm_out = fopen("a.asm", "w");
    for(i = 0; i < MAX_TABLE; i++)
        quat_table[i].label = -1;
    getsym();
    program();
	printf("请选择是否进行DAG图优化[0/1]:");
	scanf("%d", &i);
	if(i)
	{
		divide_block();
		opt_dag();
	}
    print_quat();
	toasm();
	printf("请选择是否进行窥孔优化[0/1]:");
	i= 0;
	scanf("%d", &i);
	if(i)
		opt_local();
	if(error_flag)
		printf("编译失败！详细错误请查看err.txt文件！");
	else
		printf("编译成功！\n");
	fclose(in);
	fclose(out);
	fclose(err);
	fclose(asm_out);
	if(!error_flag)
	{
		system(".\\masm32\\ml.exe /c /coff a.asm");
		system(".\\masm32\\link.exe /subsystem:console a.obj");
		printf("汇编&链接完成！是否执行？[0/1]:");
		i = 0;
		scanf("%d", &i);
		if(i)
			system("a.exe");
	}
    return 0;
}
////////////////////MAIN////////////////////
> 摘自：https://www.cnblogs.com/aclove/articles/3803110.html

**一、数据结构**

　　首先，与uthash相关的三个数据结构如下：

[![复制代码](https://assets.cnblogs.com/images/copycode.gif)](javascript:void(0);)

```c
typedef struct UT_hash_handle {
   struct UT_hash_table *tbl;
   void *prev;                       /* prev element in app order      */
   void *next;                       /* next element in app order      */
   struct UT_hash_handle *hh_prev;   /* previous hh in bucket order    */
   struct UT_hash_handle *hh_next;   /* next hh in bucket order        */
   void *key;                        /* ptr to enclosing struct's key  */
   unsigned keylen;                  /* enclosing struct's key len     */
   unsigned hashv;                   /* result of hash-fcn(key)        */
} UT_hash_handle;

typedef struct UT_hash_table {
   UT_hash_bucket *buckets;
   unsigned num_buckets, log2_num_buckets;
   unsigned num_items;
   struct UT_hash_handle *tail; /* tail hh in app order, for fast append    */
   ptrdiff_t hho; /* hash handle offset (byte pos of hash handle in element */
   unsigned ideal_chain_maxlen;
   unsigned nonideal_items;
   unsigned ineff_expands, noexpand;
   uint32_t signature; 
#ifdef HASH_BLOOM
   uint32_t bloom_sig; 
   uint8_t *bloom_bv;
   char bloom_nbits;
#endif

} UT_hash_table;


typedef struct UT_hash_bucket {
   struct UT_hash_handle *hh_head;
   unsigned count;
   unsigned expand_mult;
} UT_hash_bucket;
```

[![复制代码](https://assets.cnblogs.com/images/copycode.gif)](javascript:void(0);)

**二、初始化**

　　首先，自定义的类中必须要有一个类型为UT_hash_handle的hh成员，例如：

```c
typede struct tagHASHTEST{
    int id;
    int value;
    UT_hash_handle hh;
}HASHTEST;
```

　　然后需要定义一个头指针，并且初始化为空：

```c
HASHTEST *head = NULL;
```

 

**三、成员变量和结构体的关联**

　　**3.1 UT_hash_handle**

　　tbl：所有UT_hash_handle结构中的tble都指向同一个UT_hash_table结构；

　　prev和next：用来连接所有的HASHTEST(实际上指向的是HASHTEST中的变量hh，但由于hh是HASHTEST中的变量而不是指针，所以通过hh也可以访问到HASHTEST中的其它变量)；

　　hh_prev和hh_next：每个id都会进行hash映射，这两个变量是用来连接具有相同hash值的HASHTEST(即HASHTEST中的变量hh)；

　　key：指向对应的HASHTEST中id的指针；

　　keylen：id的大小，由于这里是整型，所以是4，实际上id可以是任意类型，为了方便，所以设置为了整型；

　　hashv：将id进行hash映射后的值，不同的id可能具有相同的hashv，不过这种概率很小，因为hashv的值都比较大。由于buckets指向的数组可能比较小（初始值为32，这个值一定是2的指数次方），所以会先对hashv进行一次按位与操作(idx = (hashv & (num_buckets-1)))，因此这个HASHTEST将被插入到buckets[idx]->hh_head指向的双向链表，这个双向链表的前后指针就是hh_prev和hh_next。

　　**3.2 UT_hash_table**

　　buckets：指向一个UT_hash_bucket数组，根据hashv的值将HASHTEST放入到相应的位置，可以看hashv的介绍；

　　num_buckets：buckets指向的数组大小；

　　log2_num_buckets：顾名思义，log2_num_buckets = log2(num_buckets)；

　　num_items：已插入到buckets中的HASHTEST总数；

　　tail：指向由prev和next链接的链表中的最后一个HASHTEST(即HASHTEST中的变量hh)；

　　hho：成员变量hh相对与HASHTEST结构体首部的位置，在这个例子中，hho=sizeof(id)+sizeof(value) = 8;

　　ideal_chain_maxlen：在理想情况下，即所有的HASHTEST刚好平坦到每个buckets[i]->hh_head指向的链表中，任何两个链表的数目相差不超过1时，一个链表中能够容纳的HASHTEST数目，实际上就等于num_items / num_buckets + (num_items % num_buckets == 0 ? 0 : 1)；

　　nonideal_items：实际上链表中HASHTEST的数目超过ideal_chain_maxlen的链表数；

　　ineff_expands：在源代码中 tbl->ineff_expands = (tbl->nonideal_items > (tbl->num_items >> 1)) ?  (tbl->ineff_expands+1) : 0；

　　noexpand：当这个值为1时，永远不会对buckets的大小进行扩充，在源代码中 if (tbl->ineff_expands > 1) tbl->noexpand=1，暂时还不知道为什么要这么做，要是有大神知道求大神回复~~，如果noexpand被置为1了，那么当以后插入元素越来越多时，而buckets指向的数组大小又不变，那么显然查找的性能会下降，而且代码中没看到将noexpand重新置0的地方；

　　signature：应该是一个幻数，这个值被设定为0xa0111fe1，可以不用管；

　　bloom_sig：幻数，这个值被设定为0xb12220f2，不用管；

　　bloom_bv：指向一个uint8_t类型的数组，用来标记buckets中每个链表是否为空，可以优化查找的速度，因为这个数组中每个元素是一个字节，所以每个元素可以标记8个链表，例如要判断bucket[1]->hh_head是否为空，只要判断(bloom_bv[0] & 2)是否为0即可;

　　bloom_nbits：bloom_bv指向的数组大小为 (1 << bloom_nbits)。

　　**3.3 UT_hash_bucket**

　　hh_head：前面已经说明；

　　count：hh_head指向的链表中的元素数目；

　　expand_mult：当count的值大于(expand_mult+1)*10时，则对buckets指向的数组的大小进行扩充；在扩充之后expand_mult被设定为count / ideal_chain_maxlen。

　　

　　成员变量之间的关系如下图所示，由于hash=128 % num_buckets = 128 % 32=0，所以第一个HASHTEST和第二个HASHTEST都被添加到buckets[0]->hh_head指向的链表中，其它同理，不详细说明，有问题可以留言~~。

 　![img](https://images0.cnblogs.com/i/638205/201406/231518349552735.jpg)

 **四、HASH操作**

 　**4.1 添加操作HASH_ADD**

　　uthash存在三个简化的添加操作HASH_ADD_STR、HASH_ADD_INT和HASH_ADD_PTR，宏定义为：

```c
#define HASH_ADD_STR(head,strfield,add) HASH_ADD(hh,head,strfield,strlen(add->strfield),add)
#define HASH_ADD_INT(head,intfield,add) HASH_ADD(hh,head,intfield,sizeof(int),add)
#define HASH_ADD_PTR(head,ptrfield,add) HASH_ADD(hh,head,ptrfield,sizeof(void *),add)
```

　　由于之前我们定义的HASHTEST中的id是整型，所以我们使用HASH_ADD_INT进行添加操作：

[![复制代码](https://assets.cnblogs.com/images/copycode.gif)](javascript:void(0);)

```c
1 for(int i=1; i<=10; ++i){
2     HASHTEST *add = (HASHTEST*)malloc(sizeof(HASHTEST));
3     if(add == NULL) exit(EXIT_FAILURE);
4     add->id = i;
5     add->value = 111;
6     HASH_ADD_INT(head, id, add);
7 }
```

[![复制代码](https://assets.cnblogs.com/images/copycode.gif)](javascript:void(0);)

 　**4.2 查找操作HASH_FIND**

 　uthash存在三个简化的查找操作HASH_FIND_STR、HASH_FIND_INT和HASH_FIND_PTR，宏定义为：

```c
#define HASH_FIND_STR(head,findstr,out) HASH_FIND(hh,head,findstr,strlen(findstr),out)
#define HASH_FIND_INT(head,findint,out) HASH_FIND(hh,head,findint,sizeof(int),out)
#define HASH_FIND_PTR(head,findptr,out) HASH_FIND(hh,head,findptr,sizeof(void *),out)
```

　　使用HASH_FIND_INT进行查找操作：

```c
1 for(int i=1; i<=10; ++i){
2     HASHTEST *tmp;
3     HASH_FIND_INT(head, &i, tmp);
4     if(tmp != NULL) printf("%d\n",tmp->value);
5 }
```

　　**4.3 替换操作HASH_REPLACE**

　　同样的，uthash存在三个简化的替换操作HASH_REPLACE_STR、HASH_REPLACE_INT和HASH_REPLACE_PTR，宏定义为：

```c
#define HASH_REPLACE_STR(head,strfield,add,replaced) HASH_REPLACE(hh,head,strfield,strlen(add-strfield),add,replaced)
#define HASH_REPLACE_INT(head,intfield,add,replaced) HASH_REPLACE(hh,head,intfield,sizeof(int),add,replaced)
#define HASH_REPLACE_PTR(head,ptrfield,add) HASH_REPLACE(hh,head,ptrfield,sizeof(void *),add,replaced)
```

　　替换操作首先查找是否存在具有相同id值的HASHTEST，如果存在则先删除这个HASHTEST，然后将需要替换的HASHTEST插入uthash中，使用HASH_REPLACE_INT进行替换操作：

[![复制代码](https://assets.cnblogs.com/images/copycode.gif)](javascript:void(0);)

```c
1 for(int i=1; i<=10; ++i){
2     HASHTEST *add, *replace;
3     add = (HASHTEST*)malloc(sizeof(HASHTEST));
4     if(add == NULL) exit(EXIT_FAILURE);
5     add->id = i;
6     add->value = 222;
7     HASH_REPLACE_INT(head, id, add, replace);
8 }
```

[![复制代码](https://assets.cnblogs.com/images/copycode.gif)](javascript:void(0);)

　　**4.4 删除操作HASH_DEL**

　　uthash中的删除操作值存在一个宏定义：

```c
#define HASH_DEL(head,delptr) HASH_DELETE(hh,head,delptr)
```

　　使用HASH_DEL进行删除操作：

```c
1 for(int i=1; i<=10; ++i){
2     HASHTEST *tmp = NULL;
3     HASH_FIND_INT(head, &i, tmp);
4     if(tmp != NULL) HASH_DEL(HEAD, tmp);
5 }
```
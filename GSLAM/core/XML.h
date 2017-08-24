/*
Original code by Lee Thomason (www.grinninglizard.com)

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any
damages arising from the use of this software.

Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and
redistribute it freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.

2. Altered source versions must be plainly marked as such, and
must not be misrepresented as being the original software.

3. This notice may not be removed or altered from any source
distribution.
*/

#ifndef TINYXML2_INCLUDED
#define TINYXML2_INCLUDED

#if defined(ANDROID_NDK) || defined(__BORLANDC__) || defined(__QNXNTO__)
#   include <ctype.h>
#   include <limits.h>
#   include <stdio.h>
#   include <stdlib.h>
#   include <string.h>
#	if defined(__PS3__)
#		include <stddef.h>
#	endif
#else
#   include <cctype>
#   include <climits>
#   include <cstdio>
#   include <cstdlib>
#   include <cstring>
#endif
#include <stdint.h>

/*
   TODO: intern strings instead of allocation.
*/
/*
	gcc:
        g++ -Wall -DDEBUG tinyxml2.cpp xmltest.cpp -o gccxmltest.exe

    Formatting, Artistic Style:
        AStyle.exe --style=1tbs --indent-switches --break-closing-brackets --indent-preprocessor tinyxml2.cpp tinyxml2.h
*/

#if defined( _DEBUG ) || defined( DEBUG ) || defined (__DEBUG__)
#   ifndef DEBUG
#       define DEBUG
#   endif
#endif

#ifdef _MSC_VER
#   pragma warning(push)
#   pragma warning(disable: 4251)
#endif

#ifdef _WIN32
#   ifdef TINYXML2_EXPORT
#       define TINYXML2_LIB __declspec(dllexport)
#   elif defined(TINYXML2_IMPORT)
#       define TINYXML2_LIB __declspec(dllimport)
#   else
#       define TINYXML2_LIB
#   endif
#elif __GNUC__ >= 4
#   define TINYXML2_LIB __attribute__((visibility("default")))
#else
#   define TINYXML2_LIB
#endif


#if defined(DEBUG)
#   if defined(_MSC_VER)
#       // "(void)0," is for suppressing C4127 warning in "assert(false)", "assert(true)" and the like
#       define TIXMLASSERT( x )           if ( !((void)0,(x))) { __debugbreak(); }
#   elif defined (ANDROID_NDK)
#       include <android/log.h>
#       define TIXMLASSERT( x )           if ( !(x)) { __android_log_assert( "assert", "grinliz", "ASSERT in '%s' at %d.", __FILE__, __LINE__ ); }
#   else
#       include <assert.h>
#       define TIXMLASSERT                assert
#   endif
#else
#   define TIXMLASSERT( x )               {}
#endif


/* Versioning, past 1.0.14:
	http://semver.org/
*/
static const int TIXML2_MAJOR_VERSION = 4;
static const int TIXML2_MINOR_VERSION = 0;
static const int TIXML2_PATCH_VERSION = 1;

namespace tinyxml2
{
class XMLDocument;
class XMLElement;
class XMLAttribute;
class XMLComment;
class XMLText;
class XMLDeclaration;
class XMLUnknown;
class XMLPrinter;

/*
	A class that wraps strings. Normally stores the start and end
	pointers into the XML file itself, and will apply normalization
	and entity translation if actually read. Can also store (and memory
	manage) a traditional char[]
*/
class StrPair
{
public:
    enum {
        NEEDS_ENTITY_PROCESSING			= 0x01,
        NEEDS_NEWLINE_NORMALIZATION		= 0x02,
        NEEDS_WHITESPACE_COLLAPSING     = 0x04,

        TEXT_ELEMENT		            = NEEDS_ENTITY_PROCESSING | NEEDS_NEWLINE_NORMALIZATION,
        TEXT_ELEMENT_LEAVE_ENTITIES		= NEEDS_NEWLINE_NORMALIZATION,
        ATTRIBUTE_NAME		            = 0,
        ATTRIBUTE_VALUE		            = NEEDS_ENTITY_PROCESSING | NEEDS_NEWLINE_NORMALIZATION,
        ATTRIBUTE_VALUE_LEAVE_ENTITIES  = NEEDS_NEWLINE_NORMALIZATION,
        COMMENT							= NEEDS_NEWLINE_NORMALIZATION
    };

    StrPair() : _flags( 0 ), _start( 0 ), _end( 0 ) {}
    ~StrPair();

    void Set( char* start, char* end, int flags ) {
        TIXMLASSERT( start );
        TIXMLASSERT( end );
        Reset();
        _start  = start;
        _end    = end;
        _flags  = flags | NEEDS_FLUSH;
    }

    const char* GetStr();

    bool Empty() const {
        return _start == _end;
    }

    void SetInternedStr( const char* str ) {
        Reset();
        _start = const_cast<char*>(str);
    }

    void SetStr( const char* str, int flags=0 );

    char* ParseText( char* in, const char* endTag, int strFlags, int* curLineNumPtr );
    char* ParseName( char* in );

    void TransferTo( StrPair* other );
	void Reset();

private:
    void CollapseWhitespace();

    enum {
        NEEDS_FLUSH = 0x100,
        NEEDS_DELETE = 0x200
    };

    int     _flags;
    char*   _start;
    char*   _end;

    StrPair( const StrPair& other );	// not supported
    void operator=( StrPair& other );	// not supported, use TransferTo()
};


/*
	A dynamic array of Plain Old Data. Doesn't support constructors, etc.
	Has a small initial memory pool, so that low or no usage will not
	cause a call to new/delete
*/
template <class T, int INITIAL_SIZE>
class DynArray
{
public:
    DynArray() {
        _mem = _pool;
        _allocated = INITIAL_SIZE;
        _size = 0;
    }

    ~DynArray() {
        if ( _mem != _pool ) {
            delete [] _mem;
        }
    }

    void Clear() {
        _size = 0;
    }

    void Push( T t ) {
        TIXMLASSERT( _size < INT_MAX );
        EnsureCapacity( _size+1 );
        _mem[_size] = t;
        ++_size;
    }

    T* PushArr( int count ) {
        TIXMLASSERT( count >= 0 );
        TIXMLASSERT( _size <= INT_MAX - count );
        EnsureCapacity( _size+count );
        T* ret = &_mem[_size];
        _size += count;
        return ret;
    }

    T Pop() {
        TIXMLASSERT( _size > 0 );
        --_size;
        return _mem[_size];
    }

    void PopArr( int count ) {
        TIXMLASSERT( _size >= count );
        _size -= count;
    }

    bool Empty() const					{
        return _size == 0;
    }

    T& operator[](int i)				{
        TIXMLASSERT( i>= 0 && i < _size );
        return _mem[i];
    }

    const T& operator[](int i) const	{
        TIXMLASSERT( i>= 0 && i < _size );
        return _mem[i];
    }

    const T& PeekTop() const            {
        TIXMLASSERT( _size > 0 );
        return _mem[ _size - 1];
    }

    int Size() const					{
        TIXMLASSERT( _size >= 0 );
        return _size;
    }

    int Capacity() const				{
        TIXMLASSERT( _allocated >= INITIAL_SIZE );
        return _allocated;
    }

    const T* Mem() const				{
        TIXMLASSERT( _mem );
        return _mem;
    }

    T* Mem()							{
        TIXMLASSERT( _mem );
        return _mem;
    }

private:
    DynArray( const DynArray& ); // not supported
    void operator=( const DynArray& ); // not supported

    void EnsureCapacity( int cap ) {
        TIXMLASSERT( cap > 0 );
        if ( cap > _allocated ) {
            TIXMLASSERT( cap <= INT_MAX / 2 );
            int newAllocated = cap * 2;
            T* newMem = new T[newAllocated];
            memcpy( newMem, _mem, sizeof(T)*_size );	// warning: not using constructors, only works for PODs
            if ( _mem != _pool ) {
                delete [] _mem;
            }
            _mem = newMem;
            _allocated = newAllocated;
        }
    }

    T*  _mem;
    T   _pool[INITIAL_SIZE];
    int _allocated;		// objects allocated
    int _size;			// number objects in use
};


/*
	Parent virtual class of a pool for fast allocation
	and deallocation of objects.
*/
class MemPool
{
public:
    MemPool() {}
    virtual ~MemPool() {}

    virtual int ItemSize() const = 0;
    virtual void* Alloc() = 0;
    virtual void Free( void* ) = 0;
    virtual void SetTracked() = 0;
    virtual void Clear() = 0;
};


/*
	Template child class to create pools of the correct type.
*/
template< int ITEM_SIZE >
class MemPoolT : public MemPool
{
public:
    MemPoolT() : _root(0), _currentAllocs(0), _nAllocs(0), _maxAllocs(0), _nUntracked(0)	{}
    ~MemPoolT() {
        Clear();
    }
    
    void Clear() {
        // Delete the blocks.
        while( !_blockPtrs.Empty()) {
            Block* b  = _blockPtrs.Pop();
            delete b;
        }
        _root = 0;
        _currentAllocs = 0;
        _nAllocs = 0;
        _maxAllocs = 0;
        _nUntracked = 0;
    }

    virtual int ItemSize() const	{
        return ITEM_SIZE;
    }
    int CurrentAllocs() const		{
        return _currentAllocs;
    }

    virtual void* Alloc() {
        if ( !_root ) {
            // Need a new block.
            Block* block = new Block();
            _blockPtrs.Push( block );

            Item* blockItems = block->items;
            for( int i = 0; i < ITEMS_PER_BLOCK - 1; ++i ) {
                blockItems[i].next = &(blockItems[i + 1]);
            }
            blockItems[ITEMS_PER_BLOCK - 1].next = 0;
            _root = blockItems;
        }
        Item* const result = _root;
        TIXMLASSERT( result != 0 );
        _root = _root->next;

        ++_currentAllocs;
        if ( _currentAllocs > _maxAllocs ) {
            _maxAllocs = _currentAllocs;
        }
        ++_nAllocs;
        ++_nUntracked;
        return result;
    }
    
    virtual void Free( void* mem ) {
        if ( !mem ) {
            return;
        }
        --_currentAllocs;
        Item* item = static_cast<Item*>( mem );
#ifdef DEBUG
        memset( item, 0xfe, sizeof( *item ) );
#endif
        item->next = _root;
        _root = item;
    }
    void Trace( const char* name ) {
        printf( "Mempool %s watermark=%d [%dk] current=%d size=%d nAlloc=%d blocks=%d\n",
                name, _maxAllocs, _maxAllocs * ITEM_SIZE / 1024, _currentAllocs,
                ITEM_SIZE, _nAllocs, _blockPtrs.Size() );
    }

    void SetTracked() {
        --_nUntracked;
    }

    int Untracked() const {
        return _nUntracked;
    }

	// This number is perf sensitive. 4k seems like a good tradeoff on my machine.
	// The test file is large, 170k.
	// Release:		VS2010 gcc(no opt)
	//		1k:		4000
	//		2k:		4000
	//		4k:		3900	21000
	//		16k:	5200
	//		32k:	4300
	//		64k:	4000	21000
    // Declared public because some compilers do not accept to use ITEMS_PER_BLOCK
    // in private part if ITEMS_PER_BLOCK is private
    enum { ITEMS_PER_BLOCK = (4 * 1024) / ITEM_SIZE };

private:
    MemPoolT( const MemPoolT& ); // not supported
    void operator=( const MemPoolT& ); // not supported

    union Item {
        Item*   next;
        char    itemData[ITEM_SIZE];
    };
    struct Block {
        Item items[ITEMS_PER_BLOCK];
    };
    DynArray< Block*, 10 > _blockPtrs;
    Item* _root;

    int _currentAllocs;
    int _nAllocs;
    int _maxAllocs;
    int _nUntracked;
};



/**
	Implements the interface to the "Visitor pattern" (see the Accept() method.)
	If you call the Accept() method, it requires being passed a XMLVisitor
	class to handle callbacks. For nodes that contain other nodes (Document, Element)
	you will get called with a VisitEnter/VisitExit pair. Nodes that are always leafs
	are simply called with Visit().

	If you return 'true' from a Visit method, recursive parsing will continue. If you return
	false, <b>no children of this node or its siblings</b> will be visited.

	All flavors of Visit methods have a default implementation that returns 'true' (continue
	visiting). You need to only override methods that are interesting to you.

	Generally Accept() is called on the XMLDocument, although all nodes support visiting.

	You should never change the document from a callback.

	@sa XMLNode::Accept()
*/
class TINYXML2_LIB XMLVisitor
{
public:
    virtual ~XMLVisitor() {}

    /// Visit a document.
    virtual bool VisitEnter( const XMLDocument& /*doc*/ )			{
        return true;
    }
    /// Visit a document.
    virtual bool VisitExit( const XMLDocument& /*doc*/ )			{
        return true;
    }

    /// Visit an element.
    virtual bool VisitEnter( const XMLElement& /*element*/, const XMLAttribute* /*firstAttribute*/ )	{
        return true;
    }
    /// Visit an element.
    virtual bool VisitExit( const XMLElement& /*element*/ )			{
        return true;
    }

    /// Visit a declaration.
    virtual bool Visit( const XMLDeclaration& /*declaration*/ )		{
        return true;
    }
    /// Visit a text node.
    virtual bool Visit( const XMLText& /*text*/ )					{
        return true;
    }
    /// Visit a comment node.
    virtual bool Visit( const XMLComment& /*comment*/ )				{
        return true;
    }
    /// Visit an unknown node.
    virtual bool Visit( const XMLUnknown& /*unknown*/ )				{
        return true;
    }
};

// WARNING: must match XMLDocument::_errorNames[]
enum XMLError {
    XML_SUCCESS = 0,
    XML_NO_ATTRIBUTE,
    XML_WRONG_ATTRIBUTE_TYPE,
    XML_ERROR_FILE_NOT_FOUND,
    XML_ERROR_FILE_COULD_NOT_BE_OPENED,
    XML_ERROR_FILE_READ_ERROR,
    XML_ERROR_ELEMENT_MISMATCH,
    XML_ERROR_PARSING_ELEMENT,
    XML_ERROR_PARSING_ATTRIBUTE,
    XML_ERROR_IDENTIFYING_TAG,
    XML_ERROR_PARSING_TEXT,
    XML_ERROR_PARSING_CDATA,
    XML_ERROR_PARSING_COMMENT,
    XML_ERROR_PARSING_DECLARATION,
    XML_ERROR_PARSING_UNKNOWN,
    XML_ERROR_EMPTY_DOCUMENT,
    XML_ERROR_MISMATCHED_ELEMENT,
    XML_ERROR_PARSING,
    XML_CAN_NOT_CONVERT_TEXT,
    XML_NO_TEXT_NODE,

	XML_ERROR_COUNT
};


/*
	Utility functionality.
*/
class XMLUtil
{
public:
    static const char* SkipWhiteSpace( const char* p, int* curLineNumPtr )	{
        TIXMLASSERT( p );

        while( IsWhiteSpace(*p) ) {
            if (curLineNumPtr && *p == '\n') {
                ++(*curLineNumPtr);
            }
            ++p;
        }
        TIXMLASSERT( p );
        return p;
    }
    static char* SkipWhiteSpace( char* p, int* curLineNumPtr )				{
        return const_cast<char*>( SkipWhiteSpace( const_cast<const char*>(p), curLineNumPtr ) );
    }

    // Anything in the high order range of UTF-8 is assumed to not be whitespace. This isn't
    // correct, but simple, and usually works.
    static bool IsWhiteSpace( char p )					{
        return !IsUTF8Continuation(p) && isspace( static_cast<unsigned char>(p) );
    }
    
    inline static bool IsNameStartChar( unsigned char ch ) {
        if ( ch >= 128 ) {
            // This is a heuristic guess in attempt to not implement Unicode-aware isalpha()
            return true;
        }
        if ( isalpha( ch ) ) {
            return true;
        }
        return ch == ':' || ch == '_';
    }
    
    inline static bool IsNameChar( unsigned char ch ) {
        return IsNameStartChar( ch )
               || isdigit( ch )
               || ch == '.'
               || ch == '-';
    }

    inline static bool StringEqual( const char* p, const char* q, int nChar=INT_MAX )  {
        if ( p == q ) {
            return true;
        }
        TIXMLASSERT( p );
        TIXMLASSERT( q );
        TIXMLASSERT( nChar >= 0 );
        return strncmp( p, q, nChar ) == 0;
    }
    
    inline static bool IsUTF8Continuation( char p ) {
        return ( p & 0x80 ) != 0;
    }

    static const char* ReadBOM( const char* p, bool* hasBOM );
    // p is the starting location,
    // the UTF-8 value of the entity will be placed in value, and length filled in.
    static const char* GetCharacterRef( const char* p, char* value, int* length );
    static void ConvertUTF32ToUTF8( unsigned long input, char* output, int* length );

    // converts primitive types to strings
    static void ToStr( int v, char* buffer, int bufferSize );
    static void ToStr( unsigned v, char* buffer, int bufferSize );
    static void ToStr( bool v, char* buffer, int bufferSize );
    static void ToStr( float v, char* buffer, int bufferSize );
    static void ToStr( double v, char* buffer, int bufferSize );
	static void ToStr(int64_t v, char* buffer, int bufferSize);

    // converts strings to primitive types
    static bool	ToInt( const char* str, int* value );
    static bool ToUnsigned( const char* str, unsigned* value );
    static bool	ToBool( const char* str, bool* value );
    static bool	ToFloat( const char* str, float* value );
    static bool ToDouble( const char* str, double* value );
	static bool ToInt64(const char* str, int64_t* value);
};


/** XMLNode is a base class for every object that is in the
	XML Document Object Model (DOM), except XMLAttributes.
	Nodes have siblings, a parent, and children which can
	be navigated. A node is always in a XMLDocument.
	The type of a XMLNode can be queried, and it can
	be cast to its more defined type.

	A XMLDocument allocates memory for all its Nodes.
	When the XMLDocument gets deleted, all its Nodes
	will also be deleted.

	@verbatim
	A Document can contain:	Element	(container or leaf)
							Comment (leaf)
							Unknown (leaf)
							Declaration( leaf )

	An Element can contain:	Element (container or leaf)
							Text	(leaf)
							Attributes (not on tree)
							Comment (leaf)
							Unknown (leaf)

	@endverbatim
*/
class TINYXML2_LIB XMLNode
{
    friend class XMLDocument;
    friend class XMLElement;
public:

    /// Get the XMLDocument that owns this XMLNode.
    const XMLDocument* GetDocument() const	{
        TIXMLASSERT( _document );
        return _document;
    }
    /// Get the XMLDocument that owns this XMLNode.
    XMLDocument* GetDocument()				{
        TIXMLASSERT( _document );
        return _document;
    }

    /// Safely cast to an Element, or null.
    virtual XMLElement*		ToElement()		{
        return 0;
    }
    /// Safely cast to Text, or null.
    virtual XMLText*		ToText()		{
        return 0;
    }
    /// Safely cast to a Comment, or null.
    virtual XMLComment*		ToComment()		{
        return 0;
    }
    /// Safely cast to a Document, or null.
    virtual XMLDocument*	ToDocument()	{
        return 0;
    }
    /// Safely cast to a Declaration, or null.
    virtual XMLDeclaration*	ToDeclaration()	{
        return 0;
    }
    /// Safely cast to an Unknown, or null.
    virtual XMLUnknown*		ToUnknown()		{
        return 0;
    }

    virtual const XMLElement*		ToElement() const		{
        return 0;
    }
    virtual const XMLText*			ToText() const			{
        return 0;
    }
    virtual const XMLComment*		ToComment() const		{
        return 0;
    }
    virtual const XMLDocument*		ToDocument() const		{
        return 0;
    }
    virtual const XMLDeclaration*	ToDeclaration() const	{
        return 0;
    }
    virtual const XMLUnknown*		ToUnknown() const		{
        return 0;
    }

    /** The meaning of 'value' changes for the specific type.
    	@verbatim
    	Document:	empty (NULL is returned, not an empty string)
    	Element:	name of the element
    	Comment:	the comment text
    	Unknown:	the tag contents
    	Text:		the text string
    	@endverbatim
    */
    const char* Value() const;

    /** Set the Value of an XML node.
    	@sa Value()
    */
    void SetValue( const char* val, bool staticMem=false );

    /// Gets the line number the node is in, if the document was parsed from a file.
    int GetLineNum() const { return _parseLineNum; }

    /// Get the parent of this node on the DOM.
    const XMLNode*	Parent() const			{
        return _parent;
    }

    XMLNode* Parent()						{
        return _parent;
    }

    /// Returns true if this node has no children.
    bool NoChildren() const					{
        return !_firstChild;
    }

    /// Get the first child node, or null if none exists.
    const XMLNode*  FirstChild() const		{
        return _firstChild;
    }

    XMLNode*		FirstChild()			{
        return _firstChild;
    }

    /** Get the first child element, or optionally the first child
        element with the specified name.
    */
    const XMLElement* FirstChildElement( const char* name = 0 ) const;

    XMLElement* FirstChildElement( const char* name = 0 )	{
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->FirstChildElement( name ));
    }

    /// Get the last child node, or null if none exists.
    const XMLNode*	LastChild() const						{
        return _lastChild;
    }

    XMLNode*		LastChild()								{
        return _lastChild;
    }

    /** Get the last child element or optionally the last child
        element with the specified name.
    */
    const XMLElement* LastChildElement( const char* name = 0 ) const;

    XMLElement* LastChildElement( const char* name = 0 )	{
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->LastChildElement(name) );
    }

    /// Get the previous (left) sibling node of this node.
    const XMLNode*	PreviousSibling() const					{
        return _prev;
    }

    XMLNode*	PreviousSibling()							{
        return _prev;
    }

    /// Get the previous (left) sibling element of this node, with an optionally supplied name.
    const XMLElement*	PreviousSiblingElement( const char* name = 0 ) const ;

    XMLElement*	PreviousSiblingElement( const char* name = 0 ) {
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->PreviousSiblingElement( name ) );
    }

    /// Get the next (right) sibling node of this node.
    const XMLNode*	NextSibling() const						{
        return _next;
    }

    XMLNode*	NextSibling()								{
        return _next;
    }

    /// Get the next (right) sibling element of this node, with an optionally supplied name.
    const XMLElement*	NextSiblingElement( const char* name = 0 ) const;

    XMLElement*	NextSiblingElement( const char* name = 0 )	{
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->NextSiblingElement( name ) );
    }

    /**
    	Add a child node as the last (right) child.
		If the child node is already part of the document,
		it is moved from its old location to the new location.
		Returns the addThis argument or 0 if the node does not
		belong to the same document.
    */
    XMLNode* InsertEndChild( XMLNode* addThis );

    XMLNode* LinkEndChild( XMLNode* addThis )	{
        return InsertEndChild( addThis );
    }
    /**
    	Add a child node as the first (left) child.
		If the child node is already part of the document,
		it is moved from its old location to the new location.
		Returns the addThis argument or 0 if the node does not
		belong to the same document.
    */
    XMLNode* InsertFirstChild( XMLNode* addThis );
    /**
    	Add a node after the specified child node.
		If the child node is already part of the document,
		it is moved from its old location to the new location.
		Returns the addThis argument or 0 if the afterThis node
		is not a child of this node, or if the node does not
		belong to the same document.
    */
    XMLNode* InsertAfterChild( XMLNode* afterThis, XMLNode* addThis );

    /**
    	Delete all the children of this node.
    */
    void DeleteChildren();

    /**
    	Delete a child of this node.
    */
    void DeleteChild( XMLNode* node );

    /**
    	Make a copy of this node, but not its children.
    	You may pass in a Document pointer that will be
    	the owner of the new Node. If the 'document' is
    	null, then the node returned will be allocated
    	from the current Document. (this->GetDocument())

    	Note: if called on a XMLDocument, this will return null.
    */
    virtual XMLNode* ShallowClone( XMLDocument* document ) const = 0;

    /**
    	Test if 2 nodes are the same, but don't test children.
    	The 2 nodes do not need to be in the same Document.

    	Note: if called on a XMLDocument, this will return false.
    */
    virtual bool ShallowEqual( const XMLNode* compare ) const = 0;

    /** Accept a hierarchical visit of the nodes in the TinyXML-2 DOM. Every node in the
    	XML tree will be conditionally visited and the host will be called back
    	via the XMLVisitor interface.

    	This is essentially a SAX interface for TinyXML-2. (Note however it doesn't re-parse
    	the XML for the callbacks, so the performance of TinyXML-2 is unchanged by using this
    	interface versus any other.)

    	The interface has been based on ideas from:

    	- http://www.saxproject.org/
    	- http://c2.com/cgi/wiki?HierarchicalVisitorPattern

    	Which are both good references for "visiting".

    	An example of using Accept():
    	@verbatim
    	XMLPrinter printer;
    	tinyxmlDoc.Accept( &printer );
    	const char* xmlcstr = printer.CStr();
    	@endverbatim
    */
    virtual bool Accept( XMLVisitor* visitor ) const = 0;

	/** 
		Set user data into the XMLNode. TinyXML-2 in 
		no way processes or interprets user data.
		It is initially 0.
	*/
	void SetUserData(void* userData)	{ _userData = userData; }

	/**
		Get user data set into the XMLNode. TinyXML-2 in
		no way processes or interprets user data.
		It is initially 0.
	*/
	void* GetUserData() const			{ return _userData; }

protected:
    XMLNode( XMLDocument* );
    virtual ~XMLNode();

    virtual char* ParseDeep( char*, StrPair*, int* );

    XMLDocument*	_document;
    XMLNode*		_parent;
    mutable StrPair	_value;
    int             _parseLineNum;

    XMLNode*		_firstChild;
    XMLNode*		_lastChild;

    XMLNode*		_prev;
    XMLNode*		_next;

	void*			_userData;

private:
    MemPool*		_memPool;
    void Unlink( XMLNode* child );
    static void DeleteNode( XMLNode* node );
    void InsertChildPreamble( XMLNode* insertThis ) const;
    const XMLElement* ToElementWithName( const char* name ) const;

    XMLNode( const XMLNode& );	// not supported
    XMLNode& operator=( const XMLNode& );	// not supported
};


/** XML text.

	Note that a text node can have child element nodes, for example:
	@verbatim
	<root>This is <b>bold</b></root>
	@endverbatim

	A text node can have 2 ways to output the next. "normal" output
	and CDATA. It will default to the mode it was parsed from the XML file and
	you generally want to leave it alone, but you can change the output mode with
	SetCData() and query it with CData().
*/
class TINYXML2_LIB XMLText : public XMLNode
{
    friend class XMLDocument;
public:
    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLText* ToText()			{
        return this;
    }
    virtual const XMLText* ToText() const	{
        return this;
    }

    /// Declare whether this should be CDATA or standard text.
    void SetCData( bool isCData )			{
        _isCData = isCData;
    }
    /// Returns true if this is a CDATA text element.
    bool CData() const						{
        return _isCData;
    }

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    XMLText( XMLDocument* doc )	: XMLNode( doc ), _isCData( false )	{}
    virtual ~XMLText()												{}

    char* ParseDeep( char*, StrPair* endTag, int* curLineNumPtr );

private:
    bool _isCData;

    XMLText( const XMLText& );	// not supported
    XMLText& operator=( const XMLText& );	// not supported
};


/** An XML Comment. */
class TINYXML2_LIB XMLComment : public XMLNode
{
    friend class XMLDocument;
public:
    virtual XMLComment*	ToComment()					{
        return this;
    }
    virtual const XMLComment* ToComment() const		{
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    XMLComment( XMLDocument* doc );
    virtual ~XMLComment();

    char* ParseDeep( char*, StrPair* endTag, int* curLineNumPtr);

private:
    XMLComment( const XMLComment& );	// not supported
    XMLComment& operator=( const XMLComment& );	// not supported
};


/** In correct XML the declaration is the first entry in the file.
	@verbatim
		<?xml version="1.0" standalone="yes"?>
	@endverbatim

	TinyXML-2 will happily read or write files without a declaration,
	however.

	The text of the declaration isn't interpreted. It is parsed
	and written as a string.
*/
class TINYXML2_LIB XMLDeclaration : public XMLNode
{
    friend class XMLDocument;
public:
    virtual XMLDeclaration*	ToDeclaration()					{
        return this;
    }
    virtual const XMLDeclaration* ToDeclaration() const		{
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    XMLDeclaration( XMLDocument* doc );
    virtual ~XMLDeclaration();

    char* ParseDeep( char*, StrPair* endTag, int* curLineNumPtr );

private:
    XMLDeclaration( const XMLDeclaration& );	// not supported
    XMLDeclaration& operator=( const XMLDeclaration& );	// not supported
};


/** Any tag that TinyXML-2 doesn't recognize is saved as an
	unknown. It is a tag of text, but should not be modified.
	It will be written back to the XML, unchanged, when the file
	is saved.

	DTD tags get thrown into XMLUnknowns.
*/
class TINYXML2_LIB XMLUnknown : public XMLNode
{
    friend class XMLDocument;
public:
    virtual XMLUnknown*	ToUnknown()					{
        return this;
    }
    virtual const XMLUnknown* ToUnknown() const		{
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    XMLUnknown( XMLDocument* doc );
    virtual ~XMLUnknown();

    char* ParseDeep( char*, StrPair* endTag, int* curLineNumPtr );

private:
    XMLUnknown( const XMLUnknown& );	// not supported
    XMLUnknown& operator=( const XMLUnknown& );	// not supported
};



/** An attribute is a name-value pair. Elements have an arbitrary
	number of attributes, each with a unique name.

	@note The attributes are not XMLNodes. You may only query the
	Next() attribute in a list.
*/
class TINYXML2_LIB XMLAttribute
{
    friend class XMLElement;
public:
    /// The name of the attribute.
    const char* Name() const;

    /// The value of the attribute.
    const char* Value() const;

    /// Gets the line number the attribute is in, if the document was parsed from a file.
    int GetLineNum() const { return _parseLineNum; }

    /// The next attribute in the list.
    const XMLAttribute* Next() const {
        return _next;
    }

    /** IntValue interprets the attribute as an integer, and returns the value.
        If the value isn't an integer, 0 will be returned. There is no error checking;
    	use QueryIntValue() if you need error checking.
    */
	int	IntValue() const {
		int i = 0;
		QueryIntValue(&i);
		return i;
	}

	int64_t Int64Value() const {
		int64_t i = 0;
		QueryInt64Value(&i);
		return i;
	}

    /// Query as an unsigned integer. See IntValue()
    unsigned UnsignedValue() const			{
        unsigned i=0;
        QueryUnsignedValue( &i );
        return i;
    }
    /// Query as a boolean. See IntValue()
    bool	 BoolValue() const				{
        bool b=false;
        QueryBoolValue( &b );
        return b;
    }
    /// Query as a double. See IntValue()
    double 	 DoubleValue() const			{
        double d=0;
        QueryDoubleValue( &d );
        return d;
    }
    /// Query as a float. See IntValue()
    float	 FloatValue() const				{
        float f=0;
        QueryFloatValue( &f );
        return f;
    }

    /** QueryIntValue interprets the attribute as an integer, and returns the value
    	in the provided parameter. The function will return XML_SUCCESS on success,
    	and XML_WRONG_ATTRIBUTE_TYPE if the conversion is not successful.
    */
    XMLError QueryIntValue( int* value ) const;
    /// See QueryIntValue
    XMLError QueryUnsignedValue( unsigned int* value ) const;
	/// See QueryIntValue
	XMLError QueryInt64Value(int64_t* value) const;
	/// See QueryIntValue
    XMLError QueryBoolValue( bool* value ) const;
    /// See QueryIntValue
    XMLError QueryDoubleValue( double* value ) const;
    /// See QueryIntValue
    XMLError QueryFloatValue( float* value ) const;

    /// Set the attribute to a string value.
    void SetAttribute( const char* value );
    /// Set the attribute to value.
    void SetAttribute( int value );
    /// Set the attribute to value.
    void SetAttribute( unsigned value );
	/// Set the attribute to value.
	void SetAttribute(int64_t value);
	/// Set the attribute to value.
    void SetAttribute( bool value );
    /// Set the attribute to value.
    void SetAttribute( double value );
    /// Set the attribute to value.
    void SetAttribute( float value );

private:
    enum { BUF_SIZE = 200 };

    XMLAttribute() : _parseLineNum( 0 ), _next( 0 ), _memPool( 0 ) {}
    virtual ~XMLAttribute()	{}

    XMLAttribute( const XMLAttribute& );	// not supported
    void operator=( const XMLAttribute& );	// not supported
    void SetName( const char* name );

    char* ParseDeep( char* p, bool processEntities, int* curLineNumPtr );

    mutable StrPair _name;
    mutable StrPair _value;
    int             _parseLineNum;
    XMLAttribute*   _next;
    MemPool*        _memPool;
};


/** The element is a container class. It has a value, the element name,
	and can contain other elements, text, comments, and unknowns.
	Elements also contain an arbitrary number of attributes.
*/
class TINYXML2_LIB XMLElement : public XMLNode
{
    friend class XMLDocument;
public:
    /// Get the name of an element (which is the Value() of the node.)
    const char* Name() const		{
        return Value();
    }
    /// Set the name of the element.
    void SetName( const char* str, bool staticMem=false )	{
        SetValue( str, staticMem );
    }

    virtual XMLElement* ToElement()				{
        return this;
    }
    virtual const XMLElement* ToElement() const {
        return this;
    }
    virtual bool Accept( XMLVisitor* visitor ) const;

    /** Given an attribute name, Attribute() returns the value
    	for the attribute of that name, or null if none
    	exists. For example:

    	@verbatim
    	const char* value = ele->Attribute( "foo" );
    	@endverbatim

    	The 'value' parameter is normally null. However, if specified,
    	the attribute will only be returned if the 'name' and 'value'
    	match. This allow you to write code:

    	@verbatim
    	if ( ele->Attribute( "foo", "bar" ) ) callFooIsBar();
    	@endverbatim

    	rather than:
    	@verbatim
    	if ( ele->Attribute( "foo" ) ) {
    		if ( strcmp( ele->Attribute( "foo" ), "bar" ) == 0 ) callFooIsBar();
    	}
    	@endverbatim
    */
    const char* Attribute( const char* name, const char* value=0 ) const;

    /** Given an attribute name, IntAttribute() returns the value
    	of the attribute interpreted as an integer. The default
        value will be returned if the attribute isn't present,
        or if there is an error. (For a method with error
    	checking, see QueryIntAttribute()).
    */
	int IntAttribute(const char* name, int defaultValue = 0) const;
    /// See IntAttribute()
	unsigned UnsignedAttribute(const char* name, unsigned defaultValue = 0) const;
	/// See IntAttribute()
	int64_t Int64Attribute(const char* name, int64_t defaultValue = 0) const;
	/// See IntAttribute()
	bool BoolAttribute(const char* name, bool defaultValue = false) const;
    /// See IntAttribute()
	double DoubleAttribute(const char* name, double defaultValue = 0) const;
    /// See IntAttribute()
	float FloatAttribute(const char* name, float defaultValue = 0) const;

    /** Given an attribute name, QueryIntAttribute() returns
    	XML_SUCCESS, XML_WRONG_ATTRIBUTE_TYPE if the conversion
    	can't be performed, or XML_NO_ATTRIBUTE if the attribute
    	doesn't exist. If successful, the result of the conversion
    	will be written to 'value'. If not successful, nothing will
    	be written to 'value'. This allows you to provide default
    	value:

    	@verbatim
    	int value = 10;
    	QueryIntAttribute( "foo", &value );		// if "foo" isn't found, value will still be 10
    	@endverbatim
    */
    XMLError QueryIntAttribute( const char* name, int* value ) const				{
        const XMLAttribute* a = FindAttribute( name );
        if ( !a ) {
            return XML_NO_ATTRIBUTE;
        }
        return a->QueryIntValue( value );
    }

	/// See QueryIntAttribute()
    XMLError QueryUnsignedAttribute( const char* name, unsigned int* value ) const	{
        const XMLAttribute* a = FindAttribute( name );
        if ( !a ) {
            return XML_NO_ATTRIBUTE;
        }
        return a->QueryUnsignedValue( value );
    }

	/// See QueryIntAttribute()
	XMLError QueryInt64Attribute(const char* name, int64_t* value) const {
		const XMLAttribute* a = FindAttribute(name);
		if (!a) {
			return XML_NO_ATTRIBUTE;
		}
		return a->QueryInt64Value(value);
	}

	/// See QueryIntAttribute()
    XMLError QueryBoolAttribute( const char* name, bool* value ) const				{
        const XMLAttribute* a = FindAttribute( name );
        if ( !a ) {
            return XML_NO_ATTRIBUTE;
        }
        return a->QueryBoolValue( value );
    }
    /// See QueryIntAttribute()
    XMLError QueryDoubleAttribute( const char* name, double* value ) const			{
        const XMLAttribute* a = FindAttribute( name );
        if ( !a ) {
            return XML_NO_ATTRIBUTE;
        }
        return a->QueryDoubleValue( value );
    }
    /// See QueryIntAttribute()
    XMLError QueryFloatAttribute( const char* name, float* value ) const			{
        const XMLAttribute* a = FindAttribute( name );
        if ( !a ) {
            return XML_NO_ATTRIBUTE;
        }
        return a->QueryFloatValue( value );
    }

	
    /** Given an attribute name, QueryAttribute() returns
    	XML_SUCCESS, XML_WRONG_ATTRIBUTE_TYPE if the conversion
    	can't be performed, or XML_NO_ATTRIBUTE if the attribute
    	doesn't exist. It is overloaded for the primitive types,
		and is a generally more convenient replacement of
		QueryIntAttribute() and related functions.
		
		If successful, the result of the conversion
    	will be written to 'value'. If not successful, nothing will
    	be written to 'value'. This allows you to provide default
    	value:

    	@verbatim
    	int value = 10;
    	QueryAttribute( "foo", &value );		// if "foo" isn't found, value will still be 10
    	@endverbatim
    */
	int QueryAttribute( const char* name, int* value ) const {
		return QueryIntAttribute( name, value );
	}

	int QueryAttribute( const char* name, unsigned int* value ) const {
		return QueryUnsignedAttribute( name, value );
	}

	int QueryAttribute(const char* name, int64_t* value) const {
		return QueryInt64Attribute(name, value);
	}

	int QueryAttribute( const char* name, bool* value ) const {
		return QueryBoolAttribute( name, value );
	}

	int QueryAttribute( const char* name, double* value ) const {
		return QueryDoubleAttribute( name, value );
	}

	int QueryAttribute( const char* name, float* value ) const {
		return QueryFloatAttribute( name, value );
	}

	/// Sets the named attribute to value.
    void SetAttribute( const char* name, const char* value )	{
        XMLAttribute* a = FindOrCreateAttribute( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char* name, int value )			{
        XMLAttribute* a = FindOrCreateAttribute( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char* name, unsigned value )		{
        XMLAttribute* a = FindOrCreateAttribute( name );
        a->SetAttribute( value );
    }

	/// Sets the named attribute to value.
	void SetAttribute(const char* name, int64_t value) {
		XMLAttribute* a = FindOrCreateAttribute(name);
		a->SetAttribute(value);
	}

	/// Sets the named attribute to value.
    void SetAttribute( const char* name, bool value )			{
        XMLAttribute* a = FindOrCreateAttribute( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char* name, double value )		{
        XMLAttribute* a = FindOrCreateAttribute( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char* name, float value )		{
        XMLAttribute* a = FindOrCreateAttribute( name );
        a->SetAttribute( value );
    }

    /**
    	Delete an attribute.
    */
    void DeleteAttribute( const char* name );

    /// Return the first attribute in the list.
    const XMLAttribute* FirstAttribute() const {
        return _rootAttribute;
    }
    /// Query a specific attribute in the list.
    const XMLAttribute* FindAttribute( const char* name ) const;

    /** Convenience function for easy access to the text inside an element. Although easy
    	and concise, GetText() is limited compared to getting the XMLText child
    	and accessing it directly.

    	If the first child of 'this' is a XMLText, the GetText()
    	returns the character string of the Text node, else null is returned.

    	This is a convenient method for getting the text of simple contained text:
    	@verbatim
    	<foo>This is text</foo>
    		const char* str = fooElement->GetText();
    	@endverbatim

    	'str' will be a pointer to "This is text".

    	Note that this function can be misleading. If the element foo was created from
    	this XML:
    	@verbatim
    		<foo><b>This is text</b></foo>
    	@endverbatim

    	then the value of str would be null. The first child node isn't a text node, it is
    	another element. From this XML:
    	@verbatim
    		<foo>This is <b>text</b></foo>
    	@endverbatim
    	GetText() will return "This is ".
    */
    const char* GetText() const;

    /** Convenience function for easy access to the text inside an element. Although easy
    	and concise, SetText() is limited compared to creating an XMLText child
    	and mutating it directly.

    	If the first child of 'this' is a XMLText, SetText() sets its value to
		the given string, otherwise it will create a first child that is an XMLText.

    	This is a convenient method for setting the text of simple contained text:
    	@verbatim
    	<foo>This is text</foo>
    		fooElement->SetText( "Hullaballoo!" );
     	<foo>Hullaballoo!</foo>
		@endverbatim

    	Note that this function can be misleading. If the element foo was created from
    	this XML:
    	@verbatim
    		<foo><b>This is text</b></foo>
    	@endverbatim

    	then it will not change "This is text", but rather prefix it with a text element:
    	@verbatim
    		<foo>Hullaballoo!<b>This is text</b></foo>
    	@endverbatim
		
		For this XML:
    	@verbatim
    		<foo />
    	@endverbatim
    	SetText() will generate
    	@verbatim
    		<foo>Hullaballoo!</foo>
    	@endverbatim
    */
	void SetText( const char* inText );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( int value );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( unsigned value );  
	/// Convenience method for setting text inside an element. See SetText() for important limitations.
	void SetText(int64_t value);
	/// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( bool value );  
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( double value );  
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( float value );  

    /**
    	Convenience method to query the value of a child text node. This is probably best
    	shown by example. Given you have a document is this form:
    	@verbatim
    		<point>
    			<x>1</x>
    			<y>1.4</y>
    		</point>
    	@endverbatim

    	The QueryIntText() and similar functions provide a safe and easier way to get to the
    	"value" of x and y.

    	@verbatim
    		int x = 0;
    		float y = 0;	// types of x and y are contrived for example
    		const XMLElement* xElement = pointElement->FirstChildElement( "x" );
    		const XMLElement* yElement = pointElement->FirstChildElement( "y" );
    		xElement->QueryIntText( &x );
    		yElement->QueryFloatText( &y );
    	@endverbatim

    	@returns XML_SUCCESS (0) on success, XML_CAN_NOT_CONVERT_TEXT if the text cannot be converted
    			 to the requested type, and XML_NO_TEXT_NODE if there is no child text to query.

    */
    XMLError QueryIntText( int* ival ) const;
    /// See QueryIntText()
    XMLError QueryUnsignedText( unsigned* uval ) const;
	/// See QueryIntText()
	XMLError QueryInt64Text(int64_t* uval) const;
	/// See QueryIntText()
    XMLError QueryBoolText( bool* bval ) const;
    /// See QueryIntText()
    XMLError QueryDoubleText( double* dval ) const;
    /// See QueryIntText()
    XMLError QueryFloatText( float* fval ) const;

	int IntText(int defaultValue = 0) const;

	/// See QueryIntText()
	unsigned UnsignedText(unsigned defaultValue = 0) const;
	/// See QueryIntText()
	int64_t Int64Text(int64_t defaultValue = 0) const;
	/// See QueryIntText()
	bool BoolText(bool defaultValue = false) const;
	/// See QueryIntText()
	double DoubleText(double defaultValue = 0) const;
	/// See QueryIntText()
	float FloatText(float defaultValue = 0) const;

    // internal:
    enum {
        OPEN,		// <foo>
        CLOSED,		// <foo/>
        CLOSING		// </foo>
    };
    int ClosingType() const {
        return _closingType;
    }
    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    char* ParseDeep( char* p, StrPair* endTag, int* curLineNumPtr );

private:
    XMLElement( XMLDocument* doc );
    virtual ~XMLElement();
    XMLElement( const XMLElement& );	// not supported
    void operator=( const XMLElement& );	// not supported

    XMLAttribute* FindAttribute( const char* name ) {
        return const_cast<XMLAttribute*>(const_cast<const XMLElement*>(this)->FindAttribute( name ));
    }
    XMLAttribute* FindOrCreateAttribute( const char* name );
    //void LinkAttribute( XMLAttribute* attrib );
    char* ParseAttributes( char* p, int* curLineNumPtr );
    static void DeleteAttribute( XMLAttribute* attribute );
    XMLAttribute* CreateAttribute();

    enum { BUF_SIZE = 200 };
    int _closingType;
    // The attribute list is ordered; there is no 'lastAttribute'
    // because the list needs to be scanned for dupes before adding
    // a new attribute.
    XMLAttribute* _rootAttribute;
};


enum Whitespace {
    PRESERVE_WHITESPACE,
    COLLAPSE_WHITESPACE
};


/** A Document binds together all the functionality.
	It can be saved, loaded, and printed to the screen.
	All Nodes are connected and allocated to a Document.
	If the Document is deleted, all its Nodes are also deleted.
*/
class TINYXML2_LIB XMLDocument : public XMLNode
{
    friend class XMLElement;
public:
    /// constructor
    XMLDocument( bool processEntities = true, Whitespace = PRESERVE_WHITESPACE );
    ~XMLDocument();

    virtual XMLDocument* ToDocument()				{
        TIXMLASSERT( this == _document );
        return this;
    }
    virtual const XMLDocument* ToDocument() const	{
        TIXMLASSERT( this == _document );
        return this;
    }

    /**
    	Parse an XML file from a character string.
    	Returns XML_SUCCESS (0) on success, or
    	an errorID.

    	You may optionally pass in the 'nBytes', which is
    	the number of bytes which will be parsed. If not
    	specified, TinyXML-2 will assume 'xml' points to a
    	null terminated string.
    */
    XMLError Parse( const char* xml, size_t nBytes=(size_t)(-1) );

    /**
    	Load an XML file from disk.
    	Returns XML_SUCCESS (0) on success, or
    	an errorID.
    */
    XMLError LoadFile( const char* filename );

    /**
    	Load an XML file from disk. You are responsible
    	for providing and closing the FILE*. 
     
        NOTE: The file should be opened as binary ("rb")
        not text in order for TinyXML-2 to correctly
        do newline normalization.

    	Returns XML_SUCCESS (0) on success, or
    	an errorID.
    */
    XMLError LoadFile( FILE* );

    /**
    	Save the XML file to disk.
    	Returns XML_SUCCESS (0) on success, or
    	an errorID.
    */
    XMLError SaveFile( const char* filename, bool compact = false );

    /**
    	Save the XML file to disk. You are responsible
    	for providing and closing the FILE*.

    	Returns XML_SUCCESS (0) on success, or
    	an errorID.
    */
    XMLError SaveFile( FILE* fp, bool compact = false );

    bool ProcessEntities() const		{
        return _processEntities;
    }
    Whitespace WhitespaceMode() const	{
        return _whitespace;
    }

    /**
    	Returns true if this document has a leading Byte Order Mark of UTF8.
    */
    bool HasBOM() const {
        return _writeBOM;
    }
    /** Sets whether to write the BOM when writing the file.
    */
    void SetBOM( bool useBOM ) {
        _writeBOM = useBOM;
    }

    /** Return the root element of DOM. Equivalent to FirstChildElement().
        To get the first node, use FirstChild().
    */
    XMLElement* RootElement()				{
        return FirstChildElement();
    }
    const XMLElement* RootElement() const	{
        return FirstChildElement();
    }

    /** Print the Document. If the Printer is not provided, it will
        print to stdout. If you provide Printer, this can print to a file:
    	@verbatim
    	XMLPrinter printer( fp );
    	doc.Print( &printer );
    	@endverbatim

    	Or you can use a printer to print to memory:
    	@verbatim
    	XMLPrinter printer;
    	doc.Print( &printer );
    	// printer.CStr() has a const char* to the XML
    	@endverbatim
    */
    void Print( XMLPrinter* streamer=0 ) const;
    virtual bool Accept( XMLVisitor* visitor ) const;

    /**
    	Create a new Element associated with
    	this Document. The memory for the Element
    	is managed by the Document.
    */
    XMLElement* NewElement( const char* name );
    /**
    	Create a new Comment associated with
    	this Document. The memory for the Comment
    	is managed by the Document.
    */
    XMLComment* NewComment( const char* comment );
    /**
    	Create a new Text associated with
    	this Document. The memory for the Text
    	is managed by the Document.
    */
    XMLText* NewText( const char* text );
    /**
    	Create a new Declaration associated with
    	this Document. The memory for the object
    	is managed by the Document.

    	If the 'text' param is null, the standard
    	declaration is used.:
    	@verbatim
    		<?xml version="1.0" encoding="UTF-8"?>
    	@endverbatim
    */
    XMLDeclaration* NewDeclaration( const char* text=0 );
    /**
    	Create a new Unknown associated with
    	this Document. The memory for the object
    	is managed by the Document.
    */
    XMLUnknown* NewUnknown( const char* text );

    /**
    	Delete a node associated with this document.
    	It will be unlinked from the DOM.
    */
    void DeleteNode( XMLNode* node );

    void SetError( XMLError error, const char* str1, const char* str2, int lineNum );

    void ClearError() {
        SetError(XML_SUCCESS, 0, 0, 0);
    }

    /// Return true if there was an error parsing the document.
    bool Error() const {
        return _errorID != XML_SUCCESS;
    }
    /// Return the errorID.
    XMLError  ErrorID() const {
        return _errorID;
    }
	const char* ErrorName() const;
    static const char* ErrorIDToName(XMLError errorID);

    /// Return a possibly helpful diagnostic location or string.
    const char* GetErrorStr1() const {
        return _errorStr1.GetStr();
    }
    /// Return a possibly helpful secondary diagnostic location or string.
    const char* GetErrorStr2() const {
        return _errorStr2.GetStr();
    }
    /// Return the line where the error occured, or zero if unknown.
    int GetErrorLineNum() const
    {
        return _errorLineNum;
    }
    /// If there is an error, print it to stdout.
    void PrintError() const;
    
    /// Clear the document, resetting it to the initial state.
    void Clear();

    // internal
    char* Identify( char* p, XMLNode** node );

    virtual XMLNode* ShallowClone( XMLDocument* /*document*/ ) const	{
        return 0;
    }
    virtual bool ShallowEqual( const XMLNode* /*compare*/ ) const	{
        return false;
    }

private:
    XMLDocument( const XMLDocument& );	// not supported
    void operator=( const XMLDocument& );	// not supported

    bool			_writeBOM;
    bool			_processEntities;
    XMLError		_errorID;
    Whitespace		_whitespace;
    mutable StrPair	_errorStr1;
    mutable StrPair	_errorStr2;
    int             _errorLineNum;
    char*			_charBuffer;
    int				_parseCurLineNum;

    MemPoolT< sizeof(XMLElement) >	 _elementPool;
    MemPoolT< sizeof(XMLAttribute) > _attributePool;
    MemPoolT< sizeof(XMLText) >		 _textPool;
    MemPoolT< sizeof(XMLComment) >	 _commentPool;


    void Parse();
};


/**
	A XMLHandle is a class that wraps a node pointer with null checks; this is
	an incredibly useful thing. Note that XMLHandle is not part of the TinyXML-2
	DOM structure. It is a separate utility class.

	Take an example:
	@verbatim
	<Document>
		<Element attributeA = "valueA">
			<Child attributeB = "value1" />
			<Child attributeB = "value2" />
		</Element>
	</Document>
	@endverbatim

	Assuming you want the value of "attributeB" in the 2nd "Child" element, it's very
	easy to write a *lot* of code that looks like:

	@verbatim
	XMLElement* root = document.FirstChildElement( "Document" );
	if ( root )
	{
		XMLElement* element = root->FirstChildElement( "Element" );
		if ( element )
		{
			XMLElement* child = element->FirstChildElement( "Child" );
			if ( child )
			{
				XMLElement* child2 = child->NextSiblingElement( "Child" );
				if ( child2 )
				{
					// Finally do something useful.
	@endverbatim

	And that doesn't even cover "else" cases. XMLHandle addresses the verbosity
	of such code. A XMLHandle checks for null pointers so it is perfectly safe
	and correct to use:

	@verbatim
	XMLHandle docHandle( &document );
	XMLElement* child2 = docHandle.FirstChildElement( "Document" ).FirstChildElement( "Element" ).FirstChildElement().NextSiblingElement();
	if ( child2 )
	{
		// do something useful
	@endverbatim

	Which is MUCH more concise and useful.

	It is also safe to copy handles - internally they are nothing more than node pointers.
	@verbatim
	XMLHandle handleCopy = handle;
	@endverbatim

	See also XMLConstHandle, which is the same as XMLHandle, but operates on const objects.
*/
class TINYXML2_LIB XMLHandle
{
public:
    /// Create a handle from any node (at any depth of the tree.) This can be a null pointer.
    XMLHandle( XMLNode* node )												{
        _node = node;
    }
    /// Create a handle from a node.
    XMLHandle( XMLNode& node )												{
        _node = &node;
    }
    /// Copy constructor
    XMLHandle( const XMLHandle& ref )										{
        _node = ref._node;
    }
    /// Assignment
    XMLHandle& operator=( const XMLHandle& ref )							{
        _node = ref._node;
        return *this;
    }

    /// Get the first child of this handle.
    XMLHandle FirstChild() 													{
        return XMLHandle( _node ? _node->FirstChild() : 0 );
    }
    /// Get the first child element of this handle.
    XMLHandle FirstChildElement( const char* name = 0 )						{
        return XMLHandle( _node ? _node->FirstChildElement( name ) : 0 );
    }
    /// Get the last child of this handle.
    XMLHandle LastChild()													{
        return XMLHandle( _node ? _node->LastChild() : 0 );
    }
    /// Get the last child element of this handle.
    XMLHandle LastChildElement( const char* name = 0 )						{
        return XMLHandle( _node ? _node->LastChildElement( name ) : 0 );
    }
    /// Get the previous sibling of this handle.
    XMLHandle PreviousSibling()												{
        return XMLHandle( _node ? _node->PreviousSibling() : 0 );
    }
    /// Get the previous sibling element of this handle.
    XMLHandle PreviousSiblingElement( const char* name = 0 )				{
        return XMLHandle( _node ? _node->PreviousSiblingElement( name ) : 0 );
    }
    /// Get the next sibling of this handle.
    XMLHandle NextSibling()													{
        return XMLHandle( _node ? _node->NextSibling() : 0 );
    }
    /// Get the next sibling element of this handle.
    XMLHandle NextSiblingElement( const char* name = 0 )					{
        return XMLHandle( _node ? _node->NextSiblingElement( name ) : 0 );
    }

    /// Safe cast to XMLNode. This can return null.
    XMLNode* ToNode()							{
        return _node;
    }
    /// Safe cast to XMLElement. This can return null.
    XMLElement* ToElement() 					{
        return ( _node ? _node->ToElement() : 0 );
    }
    /// Safe cast to XMLText. This can return null.
    XMLText* ToText() 							{
        return ( _node ? _node->ToText() : 0 );
    }
    /// Safe cast to XMLUnknown. This can return null.
    XMLUnknown* ToUnknown() 					{
        return ( _node ? _node->ToUnknown() : 0 );
    }
    /// Safe cast to XMLDeclaration. This can return null.
    XMLDeclaration* ToDeclaration() 			{
        return ( _node ? _node->ToDeclaration() : 0 );
    }

private:
    XMLNode* _node;
};


/**
	A variant of the XMLHandle class for working with const XMLNodes and Documents. It is the
	same in all regards, except for the 'const' qualifiers. See XMLHandle for API.
*/
class TINYXML2_LIB XMLConstHandle
{
public:
    XMLConstHandle( const XMLNode* node )											{
        _node = node;
    }
    XMLConstHandle( const XMLNode& node )											{
        _node = &node;
    }
    XMLConstHandle( const XMLConstHandle& ref )										{
        _node = ref._node;
    }

    XMLConstHandle& operator=( const XMLConstHandle& ref )							{
        _node = ref._node;
        return *this;
    }

    const XMLConstHandle FirstChild() const											{
        return XMLConstHandle( _node ? _node->FirstChild() : 0 );
    }
    const XMLConstHandle FirstChildElement( const char* name = 0 ) const				{
        return XMLConstHandle( _node ? _node->FirstChildElement( name ) : 0 );
    }
    const XMLConstHandle LastChild()	const										{
        return XMLConstHandle( _node ? _node->LastChild() : 0 );
    }
    const XMLConstHandle LastChildElement( const char* name = 0 ) const				{
        return XMLConstHandle( _node ? _node->LastChildElement( name ) : 0 );
    }
    const XMLConstHandle PreviousSibling() const									{
        return XMLConstHandle( _node ? _node->PreviousSibling() : 0 );
    }
    const XMLConstHandle PreviousSiblingElement( const char* name = 0 ) const		{
        return XMLConstHandle( _node ? _node->PreviousSiblingElement( name ) : 0 );
    }
    const XMLConstHandle NextSibling() const										{
        return XMLConstHandle( _node ? _node->NextSibling() : 0 );
    }
    const XMLConstHandle NextSiblingElement( const char* name = 0 ) const			{
        return XMLConstHandle( _node ? _node->NextSiblingElement( name ) : 0 );
    }


    const XMLNode* ToNode() const				{
        return _node;
    }
    const XMLElement* ToElement() const			{
        return ( _node ? _node->ToElement() : 0 );
    }
    const XMLText* ToText() const				{
        return ( _node ? _node->ToText() : 0 );
    }
    const XMLUnknown* ToUnknown() const			{
        return ( _node ? _node->ToUnknown() : 0 );
    }
    const XMLDeclaration* ToDeclaration() const	{
        return ( _node ? _node->ToDeclaration() : 0 );
    }

private:
    const XMLNode* _node;
};


/**
	Printing functionality. The XMLPrinter gives you more
	options than the XMLDocument::Print() method.

	It can:
	-# Print to memory.
	-# Print to a file you provide.
	-# Print XML without a XMLDocument.

	Print to Memory

	@verbatim
	XMLPrinter printer;
	doc.Print( &printer );
	SomeFunction( printer.CStr() );
	@endverbatim

	Print to a File

	You provide the file pointer.
	@verbatim
	XMLPrinter printer( fp );
	doc.Print( &printer );
	@endverbatim

	Print without a XMLDocument

	When loading, an XML parser is very useful. However, sometimes
	when saving, it just gets in the way. The code is often set up
	for streaming, and constructing the DOM is just overhead.

	The Printer supports the streaming case. The following code
	prints out a trivially simple XML file without ever creating
	an XML document.

	@verbatim
	XMLPrinter printer( fp );
	printer.OpenElement( "foo" );
	printer.PushAttribute( "foo", "bar" );
	printer.CloseElement();
	@endverbatim
*/
class TINYXML2_LIB XMLPrinter : public XMLVisitor
{
public:
    /** Construct the printer. If the FILE* is specified,
    	this will print to the FILE. Else it will print
    	to memory, and the result is available in CStr().
    	If 'compact' is set to true, then output is created
    	with only required whitespace and newlines.
    */
    XMLPrinter( FILE* file=0, bool compact = false, int depth = 0 );
    virtual ~XMLPrinter()	{}

    /** If streaming, write the BOM and declaration. */
    void PushHeader( bool writeBOM, bool writeDeclaration );
    /** If streaming, start writing an element.
        The element must be closed with CloseElement()
    */
    void OpenElement( const char* name, bool compactMode=false );
    /// If streaming, add an attribute to an open element.
    void PushAttribute( const char* name, const char* value );
    void PushAttribute( const char* name, int value );
    void PushAttribute( const char* name, unsigned value );
	void PushAttribute(const char* name, int64_t value);
	void PushAttribute( const char* name, bool value );
    void PushAttribute( const char* name, double value );
    /// If streaming, close the Element.
    virtual void CloseElement( bool compactMode=false );

    /// Add a text node.
    void PushText( const char* text, bool cdata=false );
    /// Add a text node from an integer.
    void PushText( int value );
    /// Add a text node from an unsigned.
    void PushText( unsigned value );
	/// Add a text node from an unsigned.
	void PushText(int64_t value);
	/// Add a text node from a bool.
    void PushText( bool value );
    /// Add a text node from a float.
    void PushText( float value );
    /// Add a text node from a double.
    void PushText( double value );

    /// Add a comment
    void PushComment( const char* comment );

    void PushDeclaration( const char* value );
    void PushUnknown( const char* value );

    virtual bool VisitEnter( const XMLDocument& /*doc*/ );
    virtual bool VisitExit( const XMLDocument& /*doc*/ )			{
        return true;
    }

    virtual bool VisitEnter( const XMLElement& element, const XMLAttribute* attribute );
    virtual bool VisitExit( const XMLElement& element );

    virtual bool Visit( const XMLText& text );
    virtual bool Visit( const XMLComment& comment );
    virtual bool Visit( const XMLDeclaration& declaration );
    virtual bool Visit( const XMLUnknown& unknown );

    /**
    	If in print to memory mode, return a pointer to
    	the XML file in memory.
    */
    const char* CStr() const {
        return _buffer.Mem();
    }
    /**
    	If in print to memory mode, return the size
    	of the XML file in memory. (Note the size returned
    	includes the terminating null.)
    */
    int CStrSize() const {
        return _buffer.Size();
    }
    /**
    	If in print to memory mode, reset the buffer to the
    	beginning.
    */
    void ClearBuffer() {
        _buffer.Clear();
        _buffer.Push(0);
    }

protected:
	virtual bool CompactMode( const XMLElement& )	{ return _compactMode; }

	/** Prints out the space before an element. You may override to change
	    the space and tabs used. A PrintSpace() override should call Print().
	*/
    virtual void PrintSpace( int depth );
    void Print( const char* format, ... );

    void SealElementIfJustOpened();
    bool _elementJustOpened;
    DynArray< const char*, 10 > _stack;

private:
    void PrintString( const char*, bool restrictedEntitySet );	// prints out, after detecting entities.

    bool _firstElement;
    FILE* _fp;
    int _depth;
    int _textDepth;
    bool _processEntities;
	bool _compactMode;

    enum {
        ENTITY_RANGE = 64,
        BUF_SIZE = 200
    };
    bool _entityFlag[ENTITY_RANGE];
    bool _restrictedEntityFlag[ENTITY_RANGE];

    DynArray< char, 20 > _buffer;
};


}	// tinyxml2

#if defined(_MSC_VER)
#   pragma warning(pop)
#endif
#include <new>		// yes, this one new style header, is in the Android SDK.
#if defined(ANDROID_NDK) || defined(__BORLANDC__) || defined(__QNXNTO__)
#   include <stddef.h>
#   include <stdarg.h>
#else
#   include <cstddef>
#   include <cstdarg>
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1400 ) && (!defined WINCE)
    // Microsoft Visual Studio, version 2005 and higher. Not WinCE.
    /*int _snprintf_s(
       char *buffer,
       size_t sizeOfBuffer,
       size_t count,
       const char *format [,
          argument] ...
    );*/
    static inline int TIXML_SNPRINTF( char* buffer, size_t size, const char* format, ... )
    {
        va_list va;
        va_start( va, format );
        int result = vsnprintf_s( buffer, size, _TRUNCATE, format, va );
        va_end( va );
        return result;
    }

    static inline int TIXML_VSNPRINTF( char* buffer, size_t size, const char* format, va_list va )
    {
        int result = vsnprintf_s( buffer, size, _TRUNCATE, format, va );
        return result;
    }

    #define TIXML_VSCPRINTF	_vscprintf
    #define TIXML_SSCANF	sscanf_s
#elif defined _MSC_VER
    // Microsoft Visual Studio 2003 and earlier or WinCE
    #define TIXML_SNPRINTF	_snprintf
    #define TIXML_VSNPRINTF _vsnprintf
    #define TIXML_SSCANF	sscanf
    #if (_MSC_VER < 1400 ) && (!defined WINCE)
        // Microsoft Visual Studio 2003 and not WinCE.
        #define TIXML_VSCPRINTF   _vscprintf // VS2003's C runtime has this, but VC6 C runtime or WinCE SDK doesn't have.
    #else
        // Microsoft Visual Studio 2003 and earlier or WinCE.
        static inline int TIXML_VSCPRINTF( const char* format, va_list va )
        {
            int len = 512;
            for (;;) {
                len = len*2;
                char* str = new char[len]();
                const int required = _vsnprintf(str, len, format, va);
                delete[] str;
                if ( required != -1 ) {
                    TIXMLASSERT( required >= 0 );
                    len = required;
                    break;
                }
            }
            TIXMLASSERT( len >= 0 );
            return len;
        }
    #endif
#else
    // GCC version 3 and higher
    //#warning( "Using sn* functions." )
    #define TIXML_SNPRINTF	snprintf
    #define TIXML_VSNPRINTF	vsnprintf
    static inline int TIXML_VSCPRINTF( const char* format, va_list va )
    {
        int len = vsnprintf( 0, 0, format, va );
        TIXMLASSERT( len >= 0 );
        return len;
    }
    #define TIXML_SSCANF   sscanf
#endif


static const char LINE_FEED				= (char)0x0a;			// all line endings are normalized to LF
static const char LF = LINE_FEED;
static const char CARRIAGE_RETURN		= (char)0x0d;			// CR gets filtered out
static const char CR = CARRIAGE_RETURN;
static const char SINGLE_QUOTE			= '\'';
static const char DOUBLE_QUOTE			= '\"';

// Bunch of unicode info at:
//		http://www.unicode.org/faq/utf_bom.html
//	ef bb bf (Microsoft "lead bytes") - designates UTF-8

static const unsigned char TIXML_UTF_LEAD_0 = 0xefU;
static const unsigned char TIXML_UTF_LEAD_1 = 0xbbU;
static const unsigned char TIXML_UTF_LEAD_2 = 0xbfU;

namespace tinyxml2
{

struct Entity {
    const char* pattern;
    int length;
    char value;
};

static const int NUM_ENTITIES = 5;
static const Entity entities[NUM_ENTITIES] = {
    { "quot", 4,	DOUBLE_QUOTE },
    { "amp", 3,		'&'  },
    { "apos", 4,	SINGLE_QUOTE },
    { "lt",	2, 		'<'	 },
    { "gt",	2,		'>'	 }
};


inline StrPair::~StrPair()
{
    Reset();
}


inline void StrPair::TransferTo( StrPair* other )
{
    if ( this == other ) {
        return;
    }
    // This in effect implements the assignment operator by "moving"
    // ownership (as in auto_ptr).

    TIXMLASSERT( other != 0 );
    TIXMLASSERT( other->_flags == 0 );
    TIXMLASSERT( other->_start == 0 );
    TIXMLASSERT( other->_end == 0 );

    other->Reset();

    other->_flags = _flags;
    other->_start = _start;
    other->_end = _end;

    _flags = 0;
    _start = 0;
    _end = 0;
}

inline void StrPair::Reset()
{
    if ( _flags & NEEDS_DELETE ) {
        delete [] _start;
    }
    _flags = 0;
    _start = 0;
    _end = 0;
}


inline void StrPair::SetStr( const char* str, int flags )
{
    TIXMLASSERT( str );
    Reset();
    size_t len = strlen( str );
    TIXMLASSERT( _start == 0 );
    _start = new char[ len+1 ];
    memcpy( _start, str, len+1 );
    _end = _start + len;
    _flags = flags | NEEDS_DELETE;
}


inline char* StrPair::ParseText( char* p, const char* endTag, int strFlags, int* curLineNumPtr )
{
    TIXMLASSERT( p );
    TIXMLASSERT( endTag && *endTag );
    TIXMLASSERT(curLineNumPtr);

    char* start = p;
    char  endChar = *endTag;
    size_t length = strlen( endTag );

    // Inner loop of text parsing.
    while ( *p ) {
        if ( *p == endChar && strncmp( p, endTag, length ) == 0 ) {
            Set( start, p, strFlags );
            return p + length;
        } else if (*p == '\n') {
            ++(*curLineNumPtr);
        }
        ++p;
        TIXMLASSERT( p );
    }
    return 0;
}


inline char* StrPair::ParseName( char* p )
{
    if ( !p || !(*p) ) {
        return 0;
    }
    if ( !XMLUtil::IsNameStartChar( *p ) ) {
        return 0;
    }

    char* const start = p;
    ++p;
    while ( *p && XMLUtil::IsNameChar( *p ) ) {
        ++p;
    }

    Set( start, p, 0 );
    return p;
}


inline void StrPair::CollapseWhitespace()
{
    // Adjusting _start would cause undefined behavior on delete[]
    TIXMLASSERT( ( _flags & NEEDS_DELETE ) == 0 );
    // Trim leading space.
    _start = XMLUtil::SkipWhiteSpace( _start, 0 );

    if ( *_start ) {
        const char* p = _start;	// the read pointer
        char* q = _start;	// the write pointer

        while( *p ) {
            if ( XMLUtil::IsWhiteSpace( *p )) {
                p = XMLUtil::SkipWhiteSpace( p, 0 );
                if ( *p == 0 ) {
                    break;    // don't write to q; this trims the trailing space.
                }
                *q = ' ';
                ++q;
            }
            *q = *p;
            ++q;
            ++p;
        }
        *q = 0;
    }
}


inline const char* StrPair::GetStr()
{
    TIXMLASSERT( _start );
    TIXMLASSERT( _end );
    if ( _flags & NEEDS_FLUSH ) {
        *_end = 0;
        _flags ^= NEEDS_FLUSH;

        if ( _flags ) {
            const char* p = _start;	// the read pointer
            char* q = _start;	// the write pointer

            while( p < _end ) {
                if ( (_flags & NEEDS_NEWLINE_NORMALIZATION) && *p == CR ) {
                    // CR-LF pair becomes LF
                    // CR alone becomes LF
                    // LF-CR becomes LF
                    if ( *(p+1) == LF ) {
                        p += 2;
                    }
                    else {
                        ++p;
                    }
                    *q = LF;
                    ++q;
                }
                else if ( (_flags & NEEDS_NEWLINE_NORMALIZATION) && *p == LF ) {
                    if ( *(p+1) == CR ) {
                        p += 2;
                    }
                    else {
                        ++p;
                    }
                    *q = LF;
                    ++q;
                }
                else if ( (_flags & NEEDS_ENTITY_PROCESSING) && *p == '&' ) {
                    // Entities handled by tinyXML2:
                    // - special entities in the entity table [in/out]
                    // - numeric character reference [in]
                    //   &#20013; or &#x4e2d;

                    if ( *(p+1) == '#' ) {
                        const int buflen = 10;
                        char buf[buflen] = { 0 };
                        int len = 0;
                        char* adjusted = const_cast<char*>( XMLUtil::GetCharacterRef( p, buf, &len ) );
                        if ( adjusted == 0 ) {
                            *q = *p;
                            ++p;
                            ++q;
                        }
                        else {
                            TIXMLASSERT( 0 <= len && len <= buflen );
                            TIXMLASSERT( q + len <= adjusted );
                            p = adjusted;
                            memcpy( q, buf, len );
                            q += len;
                        }
                    }
                    else {
                        bool entityFound = false;
                        for( int i = 0; i < NUM_ENTITIES; ++i ) {
                            const Entity& entity = entities[i];
                            if ( strncmp( p + 1, entity.pattern, entity.length ) == 0
                                    && *( p + entity.length + 1 ) == ';' ) {
                                // Found an entity - convert.
                                *q = entity.value;
                                ++q;
                                p += entity.length + 2;
                                entityFound = true;
                                break;
                            }
                        }
                        if ( !entityFound ) {
                            // fixme: treat as error?
                            ++p;
                            ++q;
                        }
                    }
                }
                else {
                    *q = *p;
                    ++p;
                    ++q;
                }
            }
            *q = 0;
        }
        // The loop below has plenty going on, and this
        // is a less useful mode. Break it out.
        if ( _flags & NEEDS_WHITESPACE_COLLAPSING ) {
            CollapseWhitespace();
        }
        _flags = (_flags & NEEDS_DELETE);
    }
    TIXMLASSERT( _start );
    return _start;
}




// --------- XMLUtil ----------- //

inline const char* XMLUtil::ReadBOM( const char* p, bool* bom )
{
    TIXMLASSERT( p );
    TIXMLASSERT( bom );
    *bom = false;
    const unsigned char* pu = reinterpret_cast<const unsigned char*>(p);
    // Check for BOM:
    if (    *(pu+0) == TIXML_UTF_LEAD_0
            && *(pu+1) == TIXML_UTF_LEAD_1
            && *(pu+2) == TIXML_UTF_LEAD_2 ) {
        *bom = true;
        p += 3;
    }
    TIXMLASSERT( p );
    return p;
}


inline void XMLUtil::ConvertUTF32ToUTF8( unsigned long input, char* output, int* length )
{
    const unsigned long BYTE_MASK = 0xBF;
    const unsigned long BYTE_MARK = 0x80;
    const unsigned long FIRST_BYTE_MARK[7] = { 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC };

    if (input < 0x80) {
        *length = 1;
    }
    else if ( input < 0x800 ) {
        *length = 2;
    }
    else if ( input < 0x10000 ) {
        *length = 3;
    }
    else if ( input < 0x200000 ) {
        *length = 4;
    }
    else {
        *length = 0;    // This code won't convert this correctly anyway.
        return;
    }

    output += *length;

    // Scary scary fall throughs.
    switch (*length) {
        case 4:
            --output;
            *output = (char)((input | BYTE_MARK) & BYTE_MASK);
            input >>= 6;
        case 3:
            --output;
            *output = (char)((input | BYTE_MARK) & BYTE_MASK);
            input >>= 6;
        case 2:
            --output;
            *output = (char)((input | BYTE_MARK) & BYTE_MASK);
            input >>= 6;
        case 1:
            --output;
            *output = (char)(input | FIRST_BYTE_MARK[*length]);
            break;
        default:
            TIXMLASSERT( false );
    }
}


inline const char* XMLUtil::GetCharacterRef( const char* p, char* value, int* length )
{
    // Presume an entity, and pull it out.
    *length = 0;

    if ( *(p+1) == '#' && *(p+2) ) {
        unsigned long ucs = 0;
        TIXMLASSERT( sizeof( ucs ) >= 4 );
        ptrdiff_t delta = 0;
        unsigned mult = 1;
        static const char SEMICOLON = ';';

        if ( *(p+2) == 'x' ) {
            // Hexadecimal.
            const char* q = p+3;
            if ( !(*q) ) {
                return 0;
            }

            q = strchr( q, SEMICOLON );

            if ( !q ) {
                return 0;
            }
            TIXMLASSERT( *q == SEMICOLON );

            delta = q-p;
            --q;

            while ( *q != 'x' ) {
                unsigned int digit = 0;

                if ( *q >= '0' && *q <= '9' ) {
                    digit = *q - '0';
                }
                else if ( *q >= 'a' && *q <= 'f' ) {
                    digit = *q - 'a' + 10;
                }
                else if ( *q >= 'A' && *q <= 'F' ) {
                    digit = *q - 'A' + 10;
                }
                else {
                    return 0;
                }
                TIXMLASSERT( digit < 16 );
                TIXMLASSERT( digit == 0 || mult <= UINT_MAX / digit );
                const unsigned int digitScaled = mult * digit;
                TIXMLASSERT( ucs <= ULONG_MAX - digitScaled );
                ucs += digitScaled;
                TIXMLASSERT( mult <= UINT_MAX / 16 );
                mult *= 16;
                --q;
            }
        }
        else {
            // Decimal.
            const char* q = p+2;
            if ( !(*q) ) {
                return 0;
            }

            q = strchr( q, SEMICOLON );

            if ( !q ) {
                return 0;
            }
            TIXMLASSERT( *q == SEMICOLON );

            delta = q-p;
            --q;

            while ( *q != '#' ) {
                if ( *q >= '0' && *q <= '9' ) {
                    const unsigned int digit = *q - '0';
                    TIXMLASSERT( digit < 10 );
                    TIXMLASSERT( digit == 0 || mult <= UINT_MAX / digit );
                    const unsigned int digitScaled = mult * digit;
                    TIXMLASSERT( ucs <= ULONG_MAX - digitScaled );
                    ucs += digitScaled;
                }
                else {
                    return 0;
                }
                TIXMLASSERT( mult <= UINT_MAX / 10 );
                mult *= 10;
                --q;
            }
        }
        // convert the UCS to UTF-8
        ConvertUTF32ToUTF8( ucs, value, length );
        return p + delta + 1;
    }
    return p+1;
}


inline void XMLUtil::ToStr( int v, char* buffer, int bufferSize )
{
    TIXML_SNPRINTF( buffer, bufferSize, "%d", v );
}


inline void XMLUtil::ToStr( unsigned v, char* buffer, int bufferSize )
{
    TIXML_SNPRINTF( buffer, bufferSize, "%u", v );
}


inline void XMLUtil::ToStr( bool v, char* buffer, int bufferSize )
{
    TIXML_SNPRINTF( buffer, bufferSize, "%s", v ? "true" : "false" );
}

/*
    ToStr() of a number is a very tricky topic.
    https://github.com/leethomason/tinyxml2/issues/106
*/
inline void XMLUtil::ToStr( float v, char* buffer, int bufferSize )
{
    TIXML_SNPRINTF( buffer, bufferSize, "%.8g", v );
}


inline void XMLUtil::ToStr( double v, char* buffer, int bufferSize )
{
    TIXML_SNPRINTF( buffer, bufferSize, "%.17g", v );
}


inline void XMLUtil::ToStr(int64_t v, char* buffer, int bufferSize)
{
    // horrible syntax trick to make the compiler happy about %lld
    TIXML_SNPRINTF(buffer, bufferSize, "%lld", (long long)v);
}


inline bool XMLUtil::ToInt( const char* str, int* value )
{
    if ( TIXML_SSCANF( str, "%d", value ) == 1 ) {
        return true;
    }
    return false;
}

inline bool XMLUtil::ToUnsigned( const char* str, unsigned *value )
{
    if ( TIXML_SSCANF( str, "%u", value ) == 1 ) {
        return true;
    }
    return false;
}

inline bool XMLUtil::ToBool( const char* str, bool* value )
{
    int ival = 0;
    if ( ToInt( str, &ival )) {
        *value = (ival==0) ? false : true;
        return true;
    }
    if ( StringEqual( str, "true" ) ) {
        *value = true;
        return true;
    }
    else if ( StringEqual( str, "false" ) ) {
        *value = false;
        return true;
    }
    return false;
}


inline bool XMLUtil::ToFloat( const char* str, float* value )
{
    if ( TIXML_SSCANF( str, "%f", value ) == 1 ) {
        return true;
    }
    return false;
}


inline bool XMLUtil::ToDouble( const char* str, double* value )
{
    if ( TIXML_SSCANF( str, "%lf", value ) == 1 ) {
        return true;
    }
    return false;
}


inline bool XMLUtil::ToInt64(const char* str, int64_t* value)
{
    long long v = 0;	// horrible syntax trick to make the compiler happy about %lld
    if (TIXML_SSCANF(str, "%lld", &v) == 1) {
        *value = (int64_t)v;
        return true;
    }
    return false;
}


inline char* XMLDocument::Identify( char* p, XMLNode** node )
{
    TIXMLASSERT( node );
    TIXMLASSERT( p );
    char* const start = p;
    int const startLine = _parseCurLineNum;
    p = XMLUtil::SkipWhiteSpace( p, &_parseCurLineNum );
    if( !*p ) {
        *node = 0;
        TIXMLASSERT( p );
        return p;
    }

    // These strings define the matching patterns:
    static const char* xmlHeader		= { "<?" };
    static const char* commentHeader	= { "<!--" };
    static const char* cdataHeader		= { "<![CDATA[" };
    static const char* dtdHeader		= { "<!" };
    static const char* elementHeader	= { "<" };	// and a header for everything else; check last.

    static const int xmlHeaderLen		= 2;
    static const int commentHeaderLen	= 4;
    static const int cdataHeaderLen		= 9;
    static const int dtdHeaderLen		= 2;
    static const int elementHeaderLen	= 1;

    TIXMLASSERT( sizeof( XMLComment ) == sizeof( XMLUnknown ) );		// use same memory pool
    TIXMLASSERT( sizeof( XMLComment ) == sizeof( XMLDeclaration ) );	// use same memory pool
    XMLNode* returnNode = 0;
    if ( XMLUtil::StringEqual( p, xmlHeader, xmlHeaderLen ) ) {
        TIXMLASSERT( sizeof( XMLDeclaration ) == _commentPool.ItemSize() );
        returnNode = new (_commentPool.Alloc()) XMLDeclaration( this );
        returnNode->_parseLineNum = _parseCurLineNum;
        returnNode->_memPool = &_commentPool;
        p += xmlHeaderLen;
    }
    else if ( XMLUtil::StringEqual( p, commentHeader, commentHeaderLen ) ) {
        TIXMLASSERT( sizeof( XMLComment ) == _commentPool.ItemSize() );
        returnNode = new (_commentPool.Alloc()) XMLComment( this );
        returnNode->_parseLineNum = _parseCurLineNum;
        returnNode->_memPool = &_commentPool;
        p += commentHeaderLen;
    }
    else if ( XMLUtil::StringEqual( p, cdataHeader, cdataHeaderLen ) ) {
        TIXMLASSERT( sizeof( XMLText ) == _textPool.ItemSize() );
        XMLText* text = new (_textPool.Alloc()) XMLText( this );
        returnNode = text;
        returnNode->_parseLineNum = _parseCurLineNum;
        returnNode->_memPool = &_textPool;
        p += cdataHeaderLen;
        text->SetCData( true );
    }
    else if ( XMLUtil::StringEqual( p, dtdHeader, dtdHeaderLen ) ) {
        TIXMLASSERT( sizeof( XMLUnknown ) == _commentPool.ItemSize() );
        returnNode = new (_commentPool.Alloc()) XMLUnknown( this );
        returnNode->_parseLineNum = _parseCurLineNum;
        returnNode->_memPool = &_commentPool;
        p += dtdHeaderLen;
    }
    else if ( XMLUtil::StringEqual( p, elementHeader, elementHeaderLen ) ) {
        TIXMLASSERT( sizeof( XMLElement ) == _elementPool.ItemSize() );
        returnNode = new (_elementPool.Alloc()) XMLElement( this );
        returnNode->_parseLineNum = _parseCurLineNum;
        returnNode->_memPool = &_elementPool;
        p += elementHeaderLen;
    }
    else {
        TIXMLASSERT( sizeof( XMLText ) == _textPool.ItemSize() );
        returnNode = new (_textPool.Alloc()) XMLText( this );
        returnNode->_memPool = &_textPool;
        returnNode->_parseLineNum = _parseCurLineNum; // Report line of first non-whitespace character
        p = start;	// Back it up, all the text counts.
        _parseCurLineNum = startLine;
    }

    TIXMLASSERT( returnNode );
    TIXMLASSERT( p );
    *node = returnNode;
    return p;
}


inline bool XMLDocument::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    if ( visitor->VisitEnter( *this ) ) {
        for ( const XMLNode* node=FirstChild(); node; node=node->NextSibling() ) {
            if ( !node->Accept( visitor ) ) {
                break;
            }
        }
    }
    return visitor->VisitExit( *this );
}


// --------- XMLNode ----------- //

inline XMLNode::XMLNode( XMLDocument* doc ) :
    _document( doc ),
    _parent( 0 ),
    _parseLineNum( 0 ),
    _firstChild( 0 ), _lastChild( 0 ),
    _prev( 0 ), _next( 0 ),
    _userData( 0 ),
    _memPool( 0 )
{
}


inline XMLNode::~XMLNode()
{
    DeleteChildren();
    if ( _parent ) {
        _parent->Unlink( this );
    }
}

inline const char* XMLNode::Value() const
{
    // Edge case: XMLDocuments don't have a Value. Return null.
    if ( this->ToDocument() )
        return 0;
    return _value.GetStr();
}

inline void XMLNode::SetValue( const char* str, bool staticMem )
{
    if ( staticMem ) {
        _value.SetInternedStr( str );
    }
    else {
        _value.SetStr( str );
    }
}


inline void XMLNode::DeleteChildren()
{
    while( _firstChild ) {
        TIXMLASSERT( _lastChild );
        DeleteChild( _firstChild );
    }
    _firstChild = _lastChild = 0;
}


inline void XMLNode::Unlink( XMLNode* child )
{
    TIXMLASSERT( child );
    TIXMLASSERT( child->_document == _document );
    TIXMLASSERT( child->_parent == this );
    if ( child == _firstChild ) {
        _firstChild = _firstChild->_next;
    }
    if ( child == _lastChild ) {
        _lastChild = _lastChild->_prev;
    }

    if ( child->_prev ) {
        child->_prev->_next = child->_next;
    }
    if ( child->_next ) {
        child->_next->_prev = child->_prev;
    }
    child->_parent = 0;
}


inline void XMLNode::DeleteChild( XMLNode* node )
{
    TIXMLASSERT( node );
    TIXMLASSERT( node->_document == _document );
    TIXMLASSERT( node->_parent == this );
    Unlink( node );
    DeleteNode( node );
}


inline XMLNode* XMLNode::InsertEndChild( XMLNode* addThis )
{
    TIXMLASSERT( addThis );
    if ( addThis->_document != _document ) {
        TIXMLASSERT( false );
        return 0;
    }
    InsertChildPreamble( addThis );

    if ( _lastChild ) {
        TIXMLASSERT( _firstChild );
        TIXMLASSERT( _lastChild->_next == 0 );
        _lastChild->_next = addThis;
        addThis->_prev = _lastChild;
        _lastChild = addThis;

        addThis->_next = 0;
    }
    else {
        TIXMLASSERT( _firstChild == 0 );
        _firstChild = _lastChild = addThis;

        addThis->_prev = 0;
        addThis->_next = 0;
    }
    addThis->_parent = this;
    return addThis;
}


inline XMLNode* XMLNode::InsertFirstChild( XMLNode* addThis )
{
    TIXMLASSERT( addThis );
    if ( addThis->_document != _document ) {
        TIXMLASSERT( false );
        return 0;
    }
    InsertChildPreamble( addThis );

    if ( _firstChild ) {
        TIXMLASSERT( _lastChild );
        TIXMLASSERT( _firstChild->_prev == 0 );

        _firstChild->_prev = addThis;
        addThis->_next = _firstChild;
        _firstChild = addThis;

        addThis->_prev = 0;
    }
    else {
        TIXMLASSERT( _lastChild == 0 );
        _firstChild = _lastChild = addThis;

        addThis->_prev = 0;
        addThis->_next = 0;
    }
    addThis->_parent = this;
    return addThis;
}


inline XMLNode* XMLNode::InsertAfterChild( XMLNode* afterThis, XMLNode* addThis )
{
    TIXMLASSERT( addThis );
    if ( addThis->_document != _document ) {
        TIXMLASSERT( false );
        return 0;
    }

    TIXMLASSERT( afterThis );

    if ( afterThis->_parent != this ) {
        TIXMLASSERT( false );
        return 0;
    }

    if ( afterThis->_next == 0 ) {
        // The last node or the only node.
        return InsertEndChild( addThis );
    }
    InsertChildPreamble( addThis );
    addThis->_prev = afterThis;
    addThis->_next = afterThis->_next;
    afterThis->_next->_prev = addThis;
    afterThis->_next = addThis;
    addThis->_parent = this;
    return addThis;
}




inline const XMLElement* XMLNode::FirstChildElement( const char* name ) const
{
    for( const XMLNode* node = _firstChild; node; node = node->_next ) {
        const XMLElement* element = node->ToElementWithName( name );
        if ( element ) {
            return element;
        }
    }
    return 0;
}


inline const XMLElement* XMLNode::LastChildElement( const char* name ) const
{
    for( const XMLNode* node = _lastChild; node; node = node->_prev ) {
        const XMLElement* element = node->ToElementWithName( name );
        if ( element ) {
            return element;
        }
    }
    return 0;
}


inline const XMLElement* XMLNode::NextSiblingElement( const char* name ) const
{
    for( const XMLNode* node = _next; node; node = node->_next ) {
        const XMLElement* element = node->ToElementWithName( name );
        if ( element ) {
            return element;
        }
    }
    return 0;
}


inline const XMLElement* XMLNode::PreviousSiblingElement( const char* name ) const
{
    for( const XMLNode* node = _prev; node; node = node->_prev ) {
        const XMLElement* element = node->ToElementWithName( name );
        if ( element ) {
            return element;
        }
    }
    return 0;
}


inline char* XMLNode::ParseDeep( char* p, StrPair* parentEnd, int* curLineNumPtr )
{
    // This is a recursive method, but thinking about it "at the current level"
    // it is a pretty simple flat list:
    //		<foo/>
    //		<!-- comment -->
    //
    // With a special case:
    //		<foo>
    //		</foo>
    //		<!-- comment -->
    //
    // Where the closing element (/foo) *must* be the next thing after the opening
    // element, and the names must match. BUT the tricky bit is that the closing
    // element will be read by the child.
    //
    // 'endTag' is the end tag for this node, it is returned by a call to a child.
    // 'parentEnd' is the end tag for the parent, which is filled in and returned.

    while( p && *p ) {
        XMLNode* node = 0;

        p = _document->Identify( p, &node );
        TIXMLASSERT( p );
        if ( node == 0 ) {
            break;
        }

        int initialLineNum = node->_parseLineNum;

        StrPair endTag;
        p = node->ParseDeep( p, &endTag, curLineNumPtr );
        if ( !p ) {
            DeleteNode( node );
            if ( !_document->Error() ) {
                _document->SetError( XML_ERROR_PARSING, 0, 0, initialLineNum);
            }
            break;
        }

        XMLDeclaration* decl = node->ToDeclaration();
        if ( decl ) {
            // Declarations are only allowed at document level
            bool wellLocated = ( ToDocument() != 0 );
            if ( wellLocated ) {
                // Multiple declarations are allowed but all declarations
                // must occur before anything else
                for ( const XMLNode* existingNode = _document->FirstChild(); existingNode; existingNode = existingNode->NextSibling() ) {
                    if ( !existingNode->ToDeclaration() ) {
                        wellLocated = false;
                        break;
                    }
                }
            }
            if ( !wellLocated ) {
                _document->SetError( XML_ERROR_PARSING_DECLARATION, decl->Value(), 0, initialLineNum);
                DeleteNode( node );
                break;
            }
        }

        XMLElement* ele = node->ToElement();
        if ( ele ) {
            // We read the end tag. Return it to the parent.
            if ( ele->ClosingType() == XMLElement::CLOSING ) {
                if ( parentEnd ) {
                    ele->_value.TransferTo( parentEnd );
                }
                node->_memPool->SetTracked();   // created and then immediately deleted.
                DeleteNode( node );
                return p;
            }

            // Handle an end tag returned to this level.
            // And handle a bunch of annoying errors.
            bool mismatch = false;
            if ( endTag.Empty() ) {
                if ( ele->ClosingType() == XMLElement::OPEN ) {
                    mismatch = true;
                }
            }
            else {
                if ( ele->ClosingType() != XMLElement::OPEN ) {
                    mismatch = true;
                }
                else if ( !XMLUtil::StringEqual( endTag.GetStr(), ele->Name() ) ) {
                    mismatch = true;
                }
            }
            if ( mismatch ) {
                _document->SetError( XML_ERROR_MISMATCHED_ELEMENT, ele->Name(), 0, initialLineNum);
                DeleteNode( node );
                break;
            }
        }
        InsertEndChild( node );
    }
    return 0;
}

inline void XMLNode::DeleteNode( XMLNode* node )
{
    if ( node == 0 ) {
        return;
    }
    MemPool* pool = node->_memPool;
    node->~XMLNode();
    pool->Free( node );
}

inline void XMLNode::InsertChildPreamble( XMLNode* insertThis ) const
{
    TIXMLASSERT( insertThis );
    TIXMLASSERT( insertThis->_document == _document );

    if ( insertThis->_parent )
        insertThis->_parent->Unlink( insertThis );
    else
        insertThis->_memPool->SetTracked();
}

inline const XMLElement* XMLNode::ToElementWithName( const char* name ) const
{
    const XMLElement* element = this->ToElement();
    if ( element == 0 ) {
        return 0;
    }
    if ( name == 0 ) {
        return element;
    }
    if ( XMLUtil::StringEqual( element->Name(), name ) ) {
       return element;
    }
    return 0;
}

// --------- XMLText ---------- //
inline char* XMLText::ParseDeep( char* p, StrPair*, int* curLineNumPtr )
{
    const char* start = p;
    if ( this->CData() ) {
        p = _value.ParseText( p, "]]>", StrPair::NEEDS_NEWLINE_NORMALIZATION, curLineNumPtr );
        if ( !p ) {
            _document->SetError( XML_ERROR_PARSING_CDATA, start, 0, _parseLineNum );
        }
        return p;
    }
    else {
        int flags = _document->ProcessEntities() ? StrPair::TEXT_ELEMENT : StrPair::TEXT_ELEMENT_LEAVE_ENTITIES;
        if ( _document->WhitespaceMode() == COLLAPSE_WHITESPACE ) {
            flags |= StrPair::NEEDS_WHITESPACE_COLLAPSING;
        }

        p = _value.ParseText( p, "<", flags, curLineNumPtr );
        if ( p && *p ) {
            return p-1;
        }
        if ( !p ) {
            _document->SetError( XML_ERROR_PARSING_TEXT, start, 0, _parseLineNum );
        }
    }
    return 0;
}


inline XMLNode* XMLText::ShallowClone( XMLDocument* doc ) const
{
    if ( !doc ) {
        doc = _document;
    }
    XMLText* text = doc->NewText( Value() );	// fixme: this will always allocate memory. Intern?
    text->SetCData( this->CData() );
    return text;
}


inline bool XMLText::ShallowEqual( const XMLNode* compare ) const
{
    const XMLText* text = compare->ToText();
    return ( text && XMLUtil::StringEqual( text->Value(), Value() ) );
}


inline bool XMLText::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}


// --------- XMLComment ---------- //

inline XMLComment::XMLComment( XMLDocument* doc ) : XMLNode( doc )
{
}


inline XMLComment::~XMLComment()
{
}


inline char* XMLComment::ParseDeep( char* p, StrPair*, int* curLineNumPtr )
{
    // Comment parses as text.
    const char* start = p;
    p = _value.ParseText( p, "-->", StrPair::COMMENT, curLineNumPtr );
    if ( p == 0 ) {
        _document->SetError( XML_ERROR_PARSING_COMMENT, start, 0, _parseLineNum );
    }
    return p;
}


inline XMLNode* XMLComment::ShallowClone( XMLDocument* doc ) const
{
    if ( !doc ) {
        doc = _document;
    }
    XMLComment* comment = doc->NewComment( Value() );	// fixme: this will always allocate memory. Intern?
    return comment;
}


inline bool XMLComment::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLComment* comment = compare->ToComment();
    return ( comment && XMLUtil::StringEqual( comment->Value(), Value() ));
}


inline bool XMLComment::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}


// --------- XMLDeclaration ---------- //

inline XMLDeclaration::XMLDeclaration( XMLDocument* doc ) : XMLNode( doc )
{
}


inline XMLDeclaration::~XMLDeclaration()
{
    //printf( "~XMLDeclaration\n" );
}


inline char* XMLDeclaration::ParseDeep( char* p, StrPair*, int* curLineNumPtr )
{
    // Declaration parses as text.
    const char* start = p;
    p = _value.ParseText( p, "?>", StrPair::NEEDS_NEWLINE_NORMALIZATION, curLineNumPtr );
    if ( p == 0 ) {
        _document->SetError( XML_ERROR_PARSING_DECLARATION, start, 0, _parseLineNum );
    }
    return p;
}


inline XMLNode* XMLDeclaration::ShallowClone( XMLDocument* doc ) const
{
    if ( !doc ) {
        doc = _document;
    }
    XMLDeclaration* dec = doc->NewDeclaration( Value() );	// fixme: this will always allocate memory. Intern?
    return dec;
}


inline bool XMLDeclaration::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLDeclaration* declaration = compare->ToDeclaration();
    return ( declaration && XMLUtil::StringEqual( declaration->Value(), Value() ));
}



inline bool XMLDeclaration::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}

// --------- XMLUnknown ---------- //

inline XMLUnknown::XMLUnknown( XMLDocument* doc ) : XMLNode( doc )
{
}


inline XMLUnknown::~XMLUnknown()
{
}


inline char* XMLUnknown::ParseDeep( char* p, StrPair*, int* curLineNumPtr )
{
    // Unknown parses as text.
    const char* start = p;

    p = _value.ParseText( p, ">", StrPair::NEEDS_NEWLINE_NORMALIZATION, curLineNumPtr );
    if ( !p ) {
        _document->SetError( XML_ERROR_PARSING_UNKNOWN, start, 0, _parseLineNum );
    }
    return p;
}


inline XMLNode* XMLUnknown::ShallowClone( XMLDocument* doc ) const
{
    if ( !doc ) {
        doc = _document;
    }
    XMLUnknown* text = doc->NewUnknown( Value() );	// fixme: this will always allocate memory. Intern?
    return text;
}


inline bool XMLUnknown::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLUnknown* unknown = compare->ToUnknown();
    return ( unknown && XMLUtil::StringEqual( unknown->Value(), Value() ));
}


inline bool XMLUnknown::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}

// --------- XMLAttribute ---------- //

inline const char* XMLAttribute::Name() const
{
    return _name.GetStr();
}

inline const char* XMLAttribute::Value() const
{
    return _value.GetStr();
}

inline char* XMLAttribute::ParseDeep( char* p, bool processEntities, int* curLineNumPtr )
{
    // Parse using the name rules: bug fix, was using ParseText before
    p = _name.ParseName( p );
    if ( !p || !*p ) {
        return 0;
    }

    // Skip white space before =
    p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );
    if ( *p != '=' ) {
        return 0;
    }

    ++p;	// move up to opening quote
    p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );
    if ( *p != '\"' && *p != '\'' ) {
        return 0;
    }

    char endTag[2] = { *p, 0 };
    ++p;	// move past opening quote

    p = _value.ParseText( p, endTag, processEntities ? StrPair::ATTRIBUTE_VALUE : StrPair::ATTRIBUTE_VALUE_LEAVE_ENTITIES, curLineNumPtr );
    return p;
}


inline void XMLAttribute::SetName( const char* n )
{
    _name.SetStr( n );
}


inline XMLError XMLAttribute::QueryIntValue( int* value ) const
{
    if ( XMLUtil::ToInt( Value(), value )) {
        return XML_SUCCESS;
    }
    return XML_WRONG_ATTRIBUTE_TYPE;
}


inline XMLError XMLAttribute::QueryUnsignedValue( unsigned int* value ) const
{
    if ( XMLUtil::ToUnsigned( Value(), value )) {
        return XML_SUCCESS;
    }
    return XML_WRONG_ATTRIBUTE_TYPE;
}


inline XMLError XMLAttribute::QueryInt64Value(int64_t* value) const
{
    if (XMLUtil::ToInt64(Value(), value)) {
        return XML_SUCCESS;
    }
    return XML_WRONG_ATTRIBUTE_TYPE;
}


inline XMLError XMLAttribute::QueryBoolValue( bool* value ) const
{
    if ( XMLUtil::ToBool( Value(), value )) {
        return XML_SUCCESS;
    }
    return XML_WRONG_ATTRIBUTE_TYPE;
}


inline XMLError XMLAttribute::QueryFloatValue( float* value ) const
{
    if ( XMLUtil::ToFloat( Value(), value )) {
        return XML_SUCCESS;
    }
    return XML_WRONG_ATTRIBUTE_TYPE;
}


inline XMLError XMLAttribute::QueryDoubleValue( double* value ) const
{
    if ( XMLUtil::ToDouble( Value(), value )) {
        return XML_SUCCESS;
    }
    return XML_WRONG_ATTRIBUTE_TYPE;
}


inline void XMLAttribute::SetAttribute( const char* v )
{
    _value.SetStr( v );
}


inline void XMLAttribute::SetAttribute( int v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    _value.SetStr( buf );
}


inline void XMLAttribute::SetAttribute( unsigned v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    _value.SetStr( buf );
}


inline void XMLAttribute::SetAttribute(int64_t v)
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr(v, buf, BUF_SIZE);
    _value.SetStr(buf);
}



inline void XMLAttribute::SetAttribute( bool v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    _value.SetStr( buf );
}

inline void XMLAttribute::SetAttribute( double v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    _value.SetStr( buf );
}

inline void XMLAttribute::SetAttribute( float v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    _value.SetStr( buf );
}


// --------- XMLElement ---------- //
inline XMLElement::XMLElement( XMLDocument* doc ) : XMLNode( doc ),
    _closingType( 0 ),
    _rootAttribute( 0 )
{
}


inline XMLElement::~XMLElement()
{
    while( _rootAttribute ) {
        XMLAttribute* next = _rootAttribute->_next;
        DeleteAttribute( _rootAttribute );
        _rootAttribute = next;
    }
}


inline const XMLAttribute* XMLElement::FindAttribute( const char* name ) const
{
    for( XMLAttribute* a = _rootAttribute; a; a = a->_next ) {
        if ( XMLUtil::StringEqual( a->Name(), name ) ) {
            return a;
        }
    }
    return 0;
}


inline const char* XMLElement::Attribute( const char* name, const char* value ) const
{
    const XMLAttribute* a = FindAttribute( name );
    if ( !a ) {
        return 0;
    }
    if ( !value || XMLUtil::StringEqual( a->Value(), value )) {
        return a->Value();
    }
    return 0;
}

inline int XMLElement::IntAttribute(const char* name, int defaultValue) const
{
    int i = defaultValue;
    QueryIntAttribute(name, &i);
    return i;
}

inline unsigned XMLElement::UnsignedAttribute(const char* name, unsigned defaultValue) const
{
    unsigned i = defaultValue;
    QueryUnsignedAttribute(name, &i);
    return i;
}

inline int64_t XMLElement::Int64Attribute(const char* name, int64_t defaultValue) const
{
    int64_t i = defaultValue;
    QueryInt64Attribute(name, &i);
    return i;
}

inline bool XMLElement::BoolAttribute(const char* name, bool defaultValue) const
{
    bool b = defaultValue;
    QueryBoolAttribute(name, &b);
    return b;
}

inline double XMLElement::DoubleAttribute(const char* name, double defaultValue) const
{
    double d = defaultValue;
    QueryDoubleAttribute(name, &d);
    return d;
}

inline float XMLElement::FloatAttribute(const char* name, float defaultValue) const
{
    float f = defaultValue;
    QueryFloatAttribute(name, &f);
    return f;
}

inline const char* XMLElement::GetText() const
{
    if ( FirstChild() && FirstChild()->ToText() ) {
        return FirstChild()->Value();
    }
    return 0;
}


inline void	XMLElement::SetText( const char* inText )
{
    if ( FirstChild() && FirstChild()->ToText() )
        FirstChild()->SetValue( inText );
    else {
        XMLText*	theText = GetDocument()->NewText( inText );
        InsertFirstChild( theText );
    }
}


inline void XMLElement::SetText( int v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    SetText( buf );
}


inline void XMLElement::SetText( unsigned v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    SetText( buf );
}


inline void XMLElement::SetText(int64_t v)
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr(v, buf, BUF_SIZE);
    SetText(buf);
}


inline void XMLElement::SetText( bool v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    SetText( buf );
}


inline void XMLElement::SetText( float v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    SetText( buf );
}


inline void XMLElement::SetText( double v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    SetText( buf );
}


inline XMLError XMLElement::QueryIntText( int* ival ) const
{
    if ( FirstChild() && FirstChild()->ToText() ) {
        const char* t = FirstChild()->Value();
        if ( XMLUtil::ToInt( t, ival ) ) {
            return XML_SUCCESS;
        }
        return XML_CAN_NOT_CONVERT_TEXT;
    }
    return XML_NO_TEXT_NODE;
}


inline XMLError XMLElement::QueryUnsignedText( unsigned* uval ) const
{
    if ( FirstChild() && FirstChild()->ToText() ) {
        const char* t = FirstChild()->Value();
        if ( XMLUtil::ToUnsigned( t, uval ) ) {
            return XML_SUCCESS;
        }
        return XML_CAN_NOT_CONVERT_TEXT;
    }
    return XML_NO_TEXT_NODE;
}


inline XMLError XMLElement::QueryInt64Text(int64_t* ival) const
{
    if (FirstChild() && FirstChild()->ToText()) {
        const char* t = FirstChild()->Value();
        if (XMLUtil::ToInt64(t, ival)) {
            return XML_SUCCESS;
        }
        return XML_CAN_NOT_CONVERT_TEXT;
    }
    return XML_NO_TEXT_NODE;
}


inline XMLError XMLElement::QueryBoolText( bool* bval ) const
{
    if ( FirstChild() && FirstChild()->ToText() ) {
        const char* t = FirstChild()->Value();
        if ( XMLUtil::ToBool( t, bval ) ) {
            return XML_SUCCESS;
        }
        return XML_CAN_NOT_CONVERT_TEXT;
    }
    return XML_NO_TEXT_NODE;
}


inline XMLError XMLElement::QueryDoubleText( double* dval ) const
{
    if ( FirstChild() && FirstChild()->ToText() ) {
        const char* t = FirstChild()->Value();
        if ( XMLUtil::ToDouble( t, dval ) ) {
            return XML_SUCCESS;
        }
        return XML_CAN_NOT_CONVERT_TEXT;
    }
    return XML_NO_TEXT_NODE;
}


inline XMLError XMLElement::QueryFloatText( float* fval ) const
{
    if ( FirstChild() && FirstChild()->ToText() ) {
        const char* t = FirstChild()->Value();
        if ( XMLUtil::ToFloat( t, fval ) ) {
            return XML_SUCCESS;
        }
        return XML_CAN_NOT_CONVERT_TEXT;
    }
    return XML_NO_TEXT_NODE;
}

inline int XMLElement::IntText(int defaultValue) const
{
    int i = defaultValue;
    QueryIntText(&i);
    return i;
}

inline unsigned XMLElement::UnsignedText(unsigned defaultValue) const
{
    unsigned i = defaultValue;
    QueryUnsignedText(&i);
    return i;
}

inline int64_t XMLElement::Int64Text(int64_t defaultValue) const
{
    int64_t i = defaultValue;
    QueryInt64Text(&i);
    return i;
}

inline bool XMLElement::BoolText(bool defaultValue) const
{
    bool b = defaultValue;
    QueryBoolText(&b);
    return b;
}

inline double XMLElement::DoubleText(double defaultValue) const
{
    double d = defaultValue;
    QueryDoubleText(&d);
    return d;
}

inline float XMLElement::FloatText(float defaultValue) const
{
    float f = defaultValue;
    QueryFloatText(&f);
    return f;
}


inline XMLAttribute* XMLElement::FindOrCreateAttribute( const char* name )
{
    XMLAttribute* last = 0;
    XMLAttribute* attrib = 0;
    for( attrib = _rootAttribute;
            attrib;
            last = attrib, attrib = attrib->_next ) {
        if ( XMLUtil::StringEqual( attrib->Name(), name ) ) {
            break;
        }
    }
    if ( !attrib ) {
        attrib = CreateAttribute();
        TIXMLASSERT( attrib );
        if ( last ) {
            TIXMLASSERT( last->_next == 0 );
            last->_next = attrib;
        }
        else {
            TIXMLASSERT( _rootAttribute == 0 );
            _rootAttribute = attrib;
        }
        attrib->SetName( name );
    }
    return attrib;
}


inline void XMLElement::DeleteAttribute( const char* name )
{
    XMLAttribute* prev = 0;
    for( XMLAttribute* a=_rootAttribute; a; a=a->_next ) {
        if ( XMLUtil::StringEqual( name, a->Name() ) ) {
            if ( prev ) {
                prev->_next = a->_next;
            }
            else {
                _rootAttribute = a->_next;
            }
            DeleteAttribute( a );
            break;
        }
        prev = a;
    }
}


inline char* XMLElement::ParseAttributes( char* p, int* curLineNumPtr )
{
    const char* start = p;
    XMLAttribute* prevAttribute = 0;

    // Read the attributes.
    while( p ) {
        p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );
        if ( !(*p) ) {
            _document->SetError( XML_ERROR_PARSING_ELEMENT, start, Name(), _parseLineNum );
            return 0;
        }

        // attribute.
        if (XMLUtil::IsNameStartChar( *p ) ) {
            XMLAttribute* attrib = CreateAttribute();
            TIXMLASSERT( attrib );
            attrib->_parseLineNum = _document->_parseCurLineNum;

            int attrLineNum = attrib->_parseLineNum;

            p = attrib->ParseDeep( p, _document->ProcessEntities(), curLineNumPtr );
            if ( !p || Attribute( attrib->Name() ) ) {
                DeleteAttribute( attrib );
                _document->SetError( XML_ERROR_PARSING_ATTRIBUTE, start, p, attrLineNum );
                return 0;
            }
            // There is a minor bug here: if the attribute in the source xml
            // document is duplicated, it will not be detected and the
            // attribute will be doubly added. However, tracking the 'prevAttribute'
            // avoids re-scanning the attribute list. Preferring performance for
            // now, may reconsider in the future.
            if ( prevAttribute ) {
                TIXMLASSERT( prevAttribute->_next == 0 );
                prevAttribute->_next = attrib;
            }
            else {
                TIXMLASSERT( _rootAttribute == 0 );
                _rootAttribute = attrib;
            }
            prevAttribute = attrib;
        }
        // end of the tag
        else if ( *p == '>' ) {
            ++p;
            break;
        }
        // end of the tag
        else if ( *p == '/' && *(p+1) == '>' ) {
            _closingType = CLOSED;
            return p+2;	// done; sealed element.
        }
        else {
            _document->SetError( XML_ERROR_PARSING_ELEMENT, start, p, _parseLineNum );
            return 0;
        }
    }
    return p;
}

inline void XMLElement::DeleteAttribute( XMLAttribute* attribute )
{
    if ( attribute == 0 ) {
        return;
    }
    MemPool* pool = attribute->_memPool;
    attribute->~XMLAttribute();
    pool->Free( attribute );
}

inline XMLAttribute* XMLElement::CreateAttribute()
{
    TIXMLASSERT( sizeof( XMLAttribute ) == _document->_attributePool.ItemSize() );
    XMLAttribute* attrib = new (_document->_attributePool.Alloc() ) XMLAttribute();
    attrib->_memPool = &_document->_attributePool;
    attrib->_memPool->SetTracked();
    return attrib;
}

//
//	<ele></ele>
//	<ele>foo<b>bar</b></ele>
//
inline char* XMLElement::ParseDeep( char* p, StrPair* strPair, int* curLineNumPtr )
{
    // Read the element name.
    p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );

    // The closing element is the </element> form. It is
    // parsed just like a regular element then deleted from
    // the DOM.
    if ( *p == '/' ) {
        _closingType = CLOSING;
        ++p;
    }

    p = _value.ParseName( p );
    if ( _value.Empty() ) {
        return 0;
    }

    p = ParseAttributes( p, curLineNumPtr );
    if ( !p || !*p || _closingType ) {
        return p;
    }

    p = XMLNode::ParseDeep( p, strPair, curLineNumPtr );
    return p;
}



inline XMLNode* XMLElement::ShallowClone( XMLDocument* doc ) const
{
    if ( !doc ) {
        doc = _document;
    }
    XMLElement* element = doc->NewElement( Value() );					// fixme: this will always allocate memory. Intern?
    for( const XMLAttribute* a=FirstAttribute(); a; a=a->Next() ) {
        element->SetAttribute( a->Name(), a->Value() );					// fixme: this will always allocate memory. Intern?
    }
    return element;
}


inline bool XMLElement::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLElement* other = compare->ToElement();
    if ( other && XMLUtil::StringEqual( other->Name(), Name() )) {

        const XMLAttribute* a=FirstAttribute();
        const XMLAttribute* b=other->FirstAttribute();

        while ( a && b ) {
            if ( !XMLUtil::StringEqual( a->Value(), b->Value() ) ) {
                return false;
            }
            a = a->Next();
            b = b->Next();
        }
        if ( a || b ) {
            // different count
            return false;
        }
        return true;
    }
    return false;
}


inline bool XMLElement::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    if ( visitor->VisitEnter( *this, _rootAttribute ) ) {
        for ( const XMLNode* node=FirstChild(); node; node=node->NextSibling() ) {
            if ( !node->Accept( visitor ) ) {
                break;
            }
        }
    }
    return visitor->VisitExit( *this );
}


// --------- XMLDocument ----------- //



inline XMLDocument::XMLDocument( bool processEntities, Whitespace whitespace ) :
    XMLNode( 0 ),
    _writeBOM( false ),
    _processEntities( processEntities ),
    _errorID(XML_SUCCESS),
    _whitespace( whitespace ),
    _errorLineNum( 0 ),
    _charBuffer( 0 ),
    _parseCurLineNum( 0 )
{
    // avoid VC++ C4355 warning about 'this' in initializer list (C4355 is off by default in VS2012+)
    _document = this;
}


inline XMLDocument::~XMLDocument()
{
    Clear();
}


inline void XMLDocument::Clear()
{
    DeleteChildren();

#ifdef DEBUG
    const bool hadError = Error();
#endif
    ClearError();

    delete [] _charBuffer;
    _charBuffer = 0;

#if 0
    _textPool.Trace( "text" );
    _elementPool.Trace( "element" );
    _commentPool.Trace( "comment" );
    _attributePool.Trace( "attribute" );
#endif

#ifdef DEBUG
    if ( !hadError ) {
        TIXMLASSERT( _elementPool.CurrentAllocs()   == _elementPool.Untracked() );
        TIXMLASSERT( _attributePool.CurrentAllocs() == _attributePool.Untracked() );
        TIXMLASSERT( _textPool.CurrentAllocs()      == _textPool.Untracked() );
        TIXMLASSERT( _commentPool.CurrentAllocs()   == _commentPool.Untracked() );
    }
#endif
}


inline XMLElement* XMLDocument::NewElement( const char* name )
{
    TIXMLASSERT( sizeof( XMLElement ) == _elementPool.ItemSize() );
    XMLElement* ele = new (_elementPool.Alloc()) XMLElement( this );
    ele->_memPool = &_elementPool;
    ele->SetName( name );
    return ele;
}


inline XMLComment* XMLDocument::NewComment( const char* str )
{
    TIXMLASSERT( sizeof( XMLComment ) == _commentPool.ItemSize() );
    XMLComment* comment = new (_commentPool.Alloc()) XMLComment( this );
    comment->_memPool = &_commentPool;
    comment->SetValue( str );
    return comment;
}


inline XMLText* XMLDocument::NewText( const char* str )
{
    TIXMLASSERT( sizeof( XMLText ) == _textPool.ItemSize() );
    XMLText* text = new (_textPool.Alloc()) XMLText( this );
    text->_memPool = &_textPool;
    text->SetValue( str );
    return text;
}


inline XMLDeclaration* XMLDocument::NewDeclaration( const char* str )
{
    TIXMLASSERT( sizeof( XMLDeclaration ) == _commentPool.ItemSize() );
    XMLDeclaration* dec = new (_commentPool.Alloc()) XMLDeclaration( this );
    dec->_memPool = &_commentPool;
    dec->SetValue( str ? str : "xml version=\"1.0\" encoding=\"UTF-8\"" );
    return dec;
}


inline XMLUnknown* XMLDocument::NewUnknown( const char* str )
{
    TIXMLASSERT( sizeof( XMLUnknown ) == _commentPool.ItemSize() );
    XMLUnknown* unk = new (_commentPool.Alloc()) XMLUnknown( this );
    unk->_memPool = &_commentPool;
    unk->SetValue( str );
    return unk;
}

inline static FILE* callfopen( const char* filepath, const char* mode )
{
    TIXMLASSERT( filepath );
    TIXMLASSERT( mode );
#if defined(_MSC_VER) && (_MSC_VER >= 1400 ) && (!defined WINCE)
    FILE* fp = 0;
    errno_t err = fopen_s( &fp, filepath, mode );
    if ( err ) {
        return 0;
    }
#else
    FILE* fp = fopen( filepath, mode );
#endif
    return fp;
}

inline void XMLDocument::DeleteNode( XMLNode* node )	{
    TIXMLASSERT( node );
    TIXMLASSERT(node->_document == this );
    if (node->_parent) {
        node->_parent->DeleteChild( node );
    }
    else {
        // Isn't in the tree.
        // Use the parent delete.
        // Also, we need to mark it tracked: we 'know'
        // it was never used.
        node->_memPool->SetTracked();
        // Call the static XMLNode version:
        XMLNode::DeleteNode(node);
    }
}


inline XMLError XMLDocument::LoadFile( const char* filename )
{
    Clear();
    FILE* fp = callfopen( filename, "rb" );
    if ( !fp ) {
        SetError( XML_ERROR_FILE_NOT_FOUND, filename, 0, 0 );
        return _errorID;
    }
    LoadFile( fp );
    fclose( fp );
    return _errorID;
}

// This is likely overengineered template art to have a check that unsigned long value incremented
// by one still fits into size_t. If size_t type is larger than unsigned long type
// (x86_64-w64-mingw32 target) then the check is redundant and gcc and clang emit
// -Wtype-limits warning. This piece makes the compiler select code with a check when a check
// is useful and code with no check when a check is redundant depending on how size_t and unsigned long
// types sizes relate to each other.
template
<bool = (sizeof(unsigned long) >= sizeof(size_t))>
struct LongFitsIntoSizeTMinusOne {
    static bool Fits( unsigned long value )
    {
        return value < (size_t)-1;
    }
};

template <>
struct LongFitsIntoSizeTMinusOne<false> {
    static bool Fits( unsigned long )
    {
        return true;
    }
};

inline XMLError XMLDocument::LoadFile( FILE* fp )
{
    Clear();

    fseek( fp, 0, SEEK_SET );
    if ( fgetc( fp ) == EOF && ferror( fp ) != 0 ) {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, 0, 0 );
        return _errorID;
    }

    fseek( fp, 0, SEEK_END );
    const long filelength = ftell( fp );
    fseek( fp, 0, SEEK_SET );
    if ( filelength == -1L ) {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, 0, 0 );
        return _errorID;
    }
    TIXMLASSERT( filelength >= 0 );

    if ( !LongFitsIntoSizeTMinusOne<>::Fits( filelength ) ) {
        // Cannot handle files which won't fit in buffer together with null terminator
        SetError( XML_ERROR_FILE_READ_ERROR, 0, 0, 0 );
        return _errorID;
    }

    if ( filelength == 0 ) {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, 0, 0 );
        return _errorID;
    }

    const size_t size = filelength;
    TIXMLASSERT( _charBuffer == 0 );
    _charBuffer = new char[size+1];
    size_t read = fread( _charBuffer, 1, size, fp );
    if ( read != size ) {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, 0, 0 );
        return _errorID;
    }

    _charBuffer[size] = 0;

    Parse();
    return _errorID;
}


inline XMLError XMLDocument::SaveFile( const char* filename, bool compact )
{
    FILE* fp = callfopen( filename, "w" );
    if ( !fp ) {
        SetError( XML_ERROR_FILE_COULD_NOT_BE_OPENED, filename, 0, 0 );
        return _errorID;
    }
    SaveFile(fp, compact);
    fclose( fp );
    return _errorID;
}


inline XMLError XMLDocument::SaveFile( FILE* fp, bool compact )
{
    // Clear any error from the last save, otherwise it will get reported
    // for *this* call.
    ClearError();
    XMLPrinter stream( fp, compact );
    Print( &stream );
    return _errorID;
}


inline XMLError XMLDocument::Parse( const char* p, size_t len )
{
    Clear();

    if ( len == 0 || !p || !*p ) {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, 0, 0 );
        return _errorID;
    }
    if ( len == (size_t)(-1) ) {
        len = strlen( p );
    }
    TIXMLASSERT( _charBuffer == 0 );
    _charBuffer = new char[ len+1 ];
    memcpy( _charBuffer, p, len );
    _charBuffer[len] = 0;

    Parse();
    if ( Error() ) {
        // clean up now essentially dangling memory.
        // and the parse fail can put objects in the
        // pools that are dead and inaccessible.
        DeleteChildren();
        _elementPool.Clear();
        _attributePool.Clear();
        _textPool.Clear();
        _commentPool.Clear();
    }
    return _errorID;
}


inline void XMLDocument::Print( XMLPrinter* streamer ) const
{
    if ( streamer ) {
        Accept( streamer );
    }
    else {
        XMLPrinter stdoutStreamer( stdout );
        Accept( &stdoutStreamer );
    }
}


inline void XMLDocument::SetError( XMLError error, const char* str1, const char* str2, int lineNum )
{
    TIXMLASSERT( error >= 0 && error < XML_ERROR_COUNT );
    _errorID = error;

    _errorStr1.Reset();
    _errorStr2.Reset();
    _errorLineNum = lineNum;

    if (str1)
        _errorStr1.SetStr(str1);
    if (str2)
        _errorStr2.SetStr(str2);
}

inline /*static*/ const char* XMLDocument::ErrorIDToName(XMLError errorID)
{
    // Warning: List must match 'enum XMLError'
    static const char* _errorNames[XML_ERROR_COUNT] = {
        "XML_SUCCESS",
        "XML_NO_ATTRIBUTE",
        "XML_WRONG_ATTRIBUTE_TYPE",
        "XML_ERROR_FILE_NOT_FOUND",
        "XML_ERROR_FILE_COULD_NOT_BE_OPENED",
        "XML_ERROR_FILE_READ_ERROR",
        "XML_ERROR_ELEMENT_MISMATCH",
        "XML_ERROR_PARSING_ELEMENT",
        "XML_ERROR_PARSING_ATTRIBUTE",
        "XML_ERROR_IDENTIFYING_TAG",
        "XML_ERROR_PARSING_TEXT",
        "XML_ERROR_PARSING_CDATA",
        "XML_ERROR_PARSING_COMMENT",
        "XML_ERROR_PARSING_DECLARATION",
        "XML_ERROR_PARSING_UNKNOWN",
        "XML_ERROR_EMPTY_DOCUMENT",
        "XML_ERROR_MISMATCHED_ELEMENT",
        "XML_ERROR_PARSING",
        "XML_CAN_NOT_CONVERT_TEXT",
        "XML_NO_TEXT_NODE"
    };

    TIXMLASSERT( errorID >= 0 && errorID < XML_ERROR_COUNT );
    const char* errorName = _errorNames[errorID];
    TIXMLASSERT( errorName && errorName[0] );
    return errorName;
}

inline const char* XMLDocument::ErrorName() const
{
    return ErrorIDToName(_errorID);
}

inline void XMLDocument::PrintError() const
{
    if ( Error() ) {
        static const int LEN = 20;
        char buf1[LEN] = { 0 };
        char buf2[LEN] = { 0 };

        if ( !_errorStr1.Empty() ) {
            TIXML_SNPRINTF( buf1, LEN, "%s", _errorStr1.GetStr() );
        }
        if ( !_errorStr2.Empty() ) {
            TIXML_SNPRINTF( buf2, LEN, "%s", _errorStr2.GetStr() );
        }

        // Should check INT_MIN <= _errorID && _errorId <= INT_MAX, but that
        // causes a clang "always true" -Wtautological-constant-out-of-range-compare warning
        TIXMLASSERT( 0 <= _errorID && XML_ERROR_COUNT - 1 <= INT_MAX );
        printf( "XMLDocument error id=%d '%s' str1=%s str2=%s line=%d\n",
                static_cast<int>( _errorID ), ErrorName(), buf1, buf2, _errorLineNum );
    }
}

inline void XMLDocument::Parse()
{
    TIXMLASSERT( NoChildren() ); // Clear() must have been called previously
    TIXMLASSERT( _charBuffer );
    _parseCurLineNum = 1;
    _parseLineNum = 1;
    char* p = _charBuffer;
    p = XMLUtil::SkipWhiteSpace( p, &_parseCurLineNum );
    p = const_cast<char*>( XMLUtil::ReadBOM( p, &_writeBOM ) );
    if ( !*p ) {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, 0, 0 );
        return;
    }
    ParseDeep(p, 0, &_parseCurLineNum );
}

inline XMLPrinter::XMLPrinter( FILE* file, bool compact, int depth ) :
    _elementJustOpened( false ),
    _firstElement( true ),
    _fp( file ),
    _depth( depth ),
    _textDepth( -1 ),
    _processEntities( true ),
    _compactMode( compact )
{
    for( int i=0; i<ENTITY_RANGE; ++i ) {
        _entityFlag[i] = false;
        _restrictedEntityFlag[i] = false;
    }
    for( int i=0; i<NUM_ENTITIES; ++i ) {
        const char entityValue = entities[i].value;
        TIXMLASSERT( ((unsigned char)entityValue) < ENTITY_RANGE );
        _entityFlag[ (unsigned char)entityValue ] = true;
    }
    _restrictedEntityFlag[(unsigned char)'&'] = true;
    _restrictedEntityFlag[(unsigned char)'<'] = true;
    _restrictedEntityFlag[(unsigned char)'>'] = true;	// not required, but consistency is nice
    _buffer.Push( 0 );
}


inline void XMLPrinter::Print( const char* format, ... )
{
    va_list     va;
    va_start( va, format );

    if ( _fp ) {
        vfprintf( _fp, format, va );
    }
    else {
        const int len = TIXML_VSCPRINTF( format, va );
        // Close out and re-start the va-args
        va_end( va );
        TIXMLASSERT( len >= 0 );
        va_start( va, format );
        TIXMLASSERT( _buffer.Size() > 0 && _buffer[_buffer.Size() - 1] == 0 );
        char* p = _buffer.PushArr( len ) - 1;	// back up over the null terminator.
        TIXML_VSNPRINTF( p, len+1, format, va );
    }
    va_end( va );
}


inline void XMLPrinter::PrintSpace( int depth )
{
    for( int i=0; i<depth; ++i ) {
        Print( "    " );
    }
}


inline void XMLPrinter::PrintString( const char* p, bool restricted )
{
    // Look for runs of bytes between entities to print.
    const char* q = p;

    if ( _processEntities ) {
        const bool* flag = restricted ? _restrictedEntityFlag : _entityFlag;
        while ( *q ) {
            TIXMLASSERT( p <= q );
            // Remember, char is sometimes signed. (How many times has that bitten me?)
            if ( *q > 0 && *q < ENTITY_RANGE ) {
                // Check for entities. If one is found, flush
                // the stream up until the entity, write the
                // entity, and keep looking.
                if ( flag[(unsigned char)(*q)] ) {
                    while ( p < q ) {
                        const size_t delta = q - p;
                        // %.*s accepts type int as "precision"
                        const int toPrint = ( INT_MAX < delta ) ? INT_MAX : (int)delta;
                        Print( "%.*s", toPrint, p );
                        p += toPrint;
                    }
                    bool entityPatternPrinted = false;
                    for( int i=0; i<NUM_ENTITIES; ++i ) {
                        if ( entities[i].value == *q ) {
                            Print( "&%s;", entities[i].pattern );
                            entityPatternPrinted = true;
                            break;
                        }
                    }
                    if ( !entityPatternPrinted ) {
                        // TIXMLASSERT( entityPatternPrinted ) causes gcc -Wunused-but-set-variable in release
                        TIXMLASSERT( false );
                    }
                    ++p;
                }
            }
            ++q;
            TIXMLASSERT( p <= q );
        }
    }
    // Flush the remaining string. This will be the entire
    // string if an entity wasn't found.
    TIXMLASSERT( p <= q );
    if ( !_processEntities || ( p < q ) ) {
        Print( "%s", p );
    }
}


inline void XMLPrinter::PushHeader( bool writeBOM, bool writeDec )
{
    if ( writeBOM ) {
        static const unsigned char bom[] = { TIXML_UTF_LEAD_0, TIXML_UTF_LEAD_1, TIXML_UTF_LEAD_2, 0 };
        Print( "%s", bom );
    }
    if ( writeDec ) {
        PushDeclaration( "xml version=\"1.0\"" );
    }
}


inline void XMLPrinter::OpenElement( const char* name, bool compactMode )
{
    SealElementIfJustOpened();
    _stack.Push( name );

    if ( _textDepth < 0 && !_firstElement && !compactMode ) {
        Print( "\n" );
    }
    if ( !compactMode ) {
        PrintSpace( _depth );
    }

    Print( "<%s", name );
    _elementJustOpened = true;
    _firstElement = false;
    ++_depth;
}


inline void XMLPrinter::PushAttribute( const char* name, const char* value )
{
    TIXMLASSERT( _elementJustOpened );
    Print( " %s=\"", name );
    PrintString( value, false );
    Print( "\"" );
}


inline void XMLPrinter::PushAttribute( const char* name, int v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    PushAttribute( name, buf );
}


inline void XMLPrinter::PushAttribute( const char* name, unsigned v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    PushAttribute( name, buf );
}


inline void XMLPrinter::PushAttribute(const char* name, int64_t v)
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr(v, buf, BUF_SIZE);
    PushAttribute(name, buf);
}


inline void XMLPrinter::PushAttribute( const char* name, bool v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    PushAttribute( name, buf );
}


inline void XMLPrinter::PushAttribute( const char* name, double v )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( v, buf, BUF_SIZE );
    PushAttribute( name, buf );
}


inline void XMLPrinter::CloseElement( bool compactMode )
{
    --_depth;
    const char* name = _stack.Pop();

    if ( _elementJustOpened ) {
        Print( "/>" );
    }
    else {
        if ( _textDepth < 0 && !compactMode) {
            Print( "\n" );
            PrintSpace( _depth );
        }
        Print( "</%s>", name );
    }

    if ( _textDepth == _depth ) {
        _textDepth = -1;
    }
    if ( _depth == 0 && !compactMode) {
        Print( "\n" );
    }
    _elementJustOpened = false;
}


inline void XMLPrinter::SealElementIfJustOpened()
{
    if ( !_elementJustOpened ) {
        return;
    }
    _elementJustOpened = false;
    Print( ">" );
}


inline void XMLPrinter::PushText( const char* text, bool cdata )
{
    _textDepth = _depth-1;

    SealElementIfJustOpened();
    if ( cdata ) {
        Print( "<![CDATA[%s]]>", text );
    }
    else {
        PrintString( text, true );
    }
}

inline void XMLPrinter::PushText( int64_t value )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( value, buf, BUF_SIZE );
    PushText( buf, false );
}

inline void XMLPrinter::PushText( int value )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( value, buf, BUF_SIZE );
    PushText( buf, false );
}


inline void XMLPrinter::PushText( unsigned value )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( value, buf, BUF_SIZE );
    PushText( buf, false );
}


inline void XMLPrinter::PushText( bool value )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( value, buf, BUF_SIZE );
    PushText( buf, false );
}


inline void XMLPrinter::PushText( float value )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( value, buf, BUF_SIZE );
    PushText( buf, false );
}


inline void XMLPrinter::PushText( double value )
{
    char buf[BUF_SIZE];
    XMLUtil::ToStr( value, buf, BUF_SIZE );
    PushText( buf, false );
}


inline void XMLPrinter::PushComment( const char* comment )
{
    SealElementIfJustOpened();
    if ( _textDepth < 0 && !_firstElement && !_compactMode) {
        Print( "\n" );
        PrintSpace( _depth );
    }
    _firstElement = false;
    Print( "<!--%s-->", comment );
}


inline void XMLPrinter::PushDeclaration( const char* value )
{
    SealElementIfJustOpened();
    if ( _textDepth < 0 && !_firstElement && !_compactMode) {
        Print( "\n" );
        PrintSpace( _depth );
    }
    _firstElement = false;
    Print( "<?%s?>", value );
}


inline void XMLPrinter::PushUnknown( const char* value )
{
    SealElementIfJustOpened();
    if ( _textDepth < 0 && !_firstElement && !_compactMode) {
        Print( "\n" );
        PrintSpace( _depth );
    }
    _firstElement = false;
    Print( "<!%s>", value );
}


inline bool XMLPrinter::VisitEnter( const XMLDocument& doc )
{
    _processEntities = doc.ProcessEntities();
    if ( doc.HasBOM() ) {
        PushHeader( true, false );
    }
    return true;
}


inline bool XMLPrinter::VisitEnter( const XMLElement& element, const XMLAttribute* attribute )
{
    const XMLElement* parentElem = 0;
    if ( element.Parent() ) {
        parentElem = element.Parent()->ToElement();
    }
    const bool compactMode = parentElem ? CompactMode( *parentElem ) : _compactMode;
    OpenElement( element.Name(), compactMode );
    while ( attribute ) {
        PushAttribute( attribute->Name(), attribute->Value() );
        attribute = attribute->Next();
    }
    return true;
}


inline bool XMLPrinter::VisitExit( const XMLElement& element )
{
    CloseElement( CompactMode(element) );
    return true;
}


inline bool XMLPrinter::Visit( const XMLText& text )
{
    PushText( text.Value(), text.CData() );
    return true;
}


inline bool XMLPrinter::Visit( const XMLComment& comment )
{
    PushComment( comment.Value() );
    return true;
}

inline bool XMLPrinter::Visit( const XMLDeclaration& declaration )
{
    PushDeclaration( declaration.Value() );
    return true;
}


inline bool XMLPrinter::Visit( const XMLUnknown& unknown )
{
    PushUnknown( unknown.Value() );
    return true;
}

}   // namespace tinyxml2

namespace GSLAM {
using namespace tinyxml2;
}
#endif // TINYXML2_INCLUDED

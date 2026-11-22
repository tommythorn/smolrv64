// THE KILL, AS A POSITION. While smolrv64_rob's kill is in force (kd_v), an op is dead when its
// ROB entry is younger than the walked-to branch: farther from kh, the head row's first tag when
// the walk began, than the branch's tag kb is. Every structure that holds an op tests its own entry
// here, with two registers and two compares side by side, instead of indexing the ROB's kill vector;
// smolrv64_rob holds this equal to its kd bit for every entry. A tag at or above kh has not wrapped
// past the ring's end since the head (kh's low bits are zero, so the tags' high bits decide): of two
// tags on one side of the wrap the greater is the farther, and of two on different sides the wrapped
// one. Included inside a module; needs ROBB.
function kdead(input k_on, input [ROBB-1:0] k_head, input [ROBB-1:0] k_br, input [ROBB-1:0] k_rob);
   reg k_nx, k_nb;
   begin
      k_nx  = k_rob >= k_head;
      k_nb  = k_br  >= k_head;
      kdead = k_on & ((k_nx == k_nb) ? (k_rob > k_br) : k_nb);
   end
endfunction

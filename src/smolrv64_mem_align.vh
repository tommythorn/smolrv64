// Load/store byte-steering helpers for the smolrv64 core. Pure combinational
// functions of their inputs; `include inside the module body.

   // Merge new_word's bytes into old_word where byte_mask is set.
   function [63:0] merge_store_bytes;
      input [63:0] old_word;
      input [63:0] new_word;
      input [ 7:0] byte_mask;
      integer byte_i;
      begin
         merge_store_bytes = old_word;
         for (byte_i = 0; byte_i < 8; byte_i = byte_i + 1)
            if (byte_mask[byte_i])
               merge_store_bytes[byte_i*8 +: 8] = new_word[byte_i*8 +: 8];
      end
   endfunction

   // Extract/sign- or zero-extend a load result. load_size encodes width and
   // signedness: 0..3 = zero-extended B/H/W/D, 4..6 = sign-extended B/H/W.
   function [63:0] align_dmem_load_value;
      input [127:0] load_data;
      input [ 2:0]  load_size;
      begin
         case (load_size)
           3'd0: align_dmem_load_value = {56'd0, load_data[7:0]};
           3'd1: align_dmem_load_value = {48'd0, load_data[15:0]};
           3'd2: align_dmem_load_value = {32'd0, load_data[31:0]};
           3'd3: align_dmem_load_value = load_data[63:0];
           3'd4: align_dmem_load_value = {{56{load_data[7]}},  load_data[7:0]};
           3'd5: align_dmem_load_value = {{48{load_data[15]}}, load_data[15:0]};
           3'd6: align_dmem_load_value = {{32{load_data[31]}}, load_data[31:0]};
           default: align_dmem_load_value = 64'd0;
         endcase
      end
   endfunction
